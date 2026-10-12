#include "Limelight-internal.h"

#define FIRST_FRAME_MAX 1500
#define FIRST_FRAME_TIMEOUT_SEC 10

#define FIRST_FRAME_PORT 47996

static RTP_VIDEO_QUEUE rtpQueue;

static SOCKET rtpSocket = INVALID_SOCKET;
static SOCKET firstFrameSocket = INVALID_SOCKET;

static PPLT_CRYPTO_CONTEXT decryptionCtx;

static PLT_THREAD udpPingThread;
static PLT_THREAD receiveThread;
static PLT_THREAD decoderThread;

static bool receivedDataFromPeer;
static uint64_t firstDataTimeMs;
static bool receivedFullFrame;

// We can't request an IDR frame until the depacketizer knows
// that a packet was lost. This timeout bounds the time that
// the RTP queue will wait for missing/reordered packets.
#define RTP_QUEUE_DELAY 10

// This is the desired number of video packets that can be
// stored in the socket's receive buffer. 2048 is chosen
// because it should be large enough for all reasonable
// frame sizes (probably 2 or 3 frames) without using too
// much kernel memory with larger packet sizes. It also
// can smooth over transient pauses in network traffic
// and subsequent packet/frame bursts that follow.
#define RTP_RECV_PACKETS_BUFFERED 2048

// Where receives can't be made non-blocking per call (Windows), how close to an unfinished
// frame's deadline the receive thread starts waking up for it even when no packets arrive (see
// LiSetPartialFrameDeadline()). Further out, it relies on the frame's own packets to wake it,
// which saves a select() for every packet. Elsewhere it always wakes for the deadline: packets
// often stall just before it, and the frame was then cut tens of ms late.
#define PARTIAL_DEADLINE_POLL_WINDOW_US 2000

// Set by LiSetPartialFrameDeadline() from the renderer, read by the receive thread
static PLT_MUTEX partialDeadlineLock;
static bool partialDeadlineEnabled;
static int64_t partialDeadlineOffsetUs;
static int partialDeadlineMinPercent;

// Clock sync with this fork's Sunshine (see SS_CLOCK_SYNC_REQUEST). Requests go out from the
// ping thread and replies come in on the receive thread, among the video packets.
#define CLOCK_SYNC_INTERVAL_MS 50

// The offset comes from the fastest of this many recent exchanges. Queueing and scheduling
// only ever make an exchange slower, so the fastest is the least disturbed. At 50 ms apart
// they span half a second, over which clocks 200 ppm apart drift 0.1 ms.
#define CLOCK_SYNC_SAMPLES 10

// Requests stop if none has been answered by then, as with any other host
#define CLOCK_SYNC_GIVE_UP_MS 3000

// Exchanges slower than this say nothing useful
#define CLOCK_SYNC_MAX_RTT_US 1000000

// Requests whose send times are kept to check replies against
#define CLOCK_SYNC_SENT_SLOTS 16

// RTP video timestamps are 32 bits at 90 kHz, so presentationTimeUs wraps after this long
#define RTP_TIMESTAMP_PERIOD_US (4294967296.0 * 1000.0 / 90.0)

typedef struct _CLOCK_SYNC_SAMPLE {
    int64_t rttUs;
    int64_t offsetUs; // The host's clock minus ours
    uint64_t hostUs; // When the host received the request
} CLOCK_SYNC_SAMPLE;

static PLT_MUTEX clockSyncLock;
static CLOCK_SYNC_SAMPLE clockSyncSamples[CLOCK_SYNC_SAMPLES];
static int clockSyncSampleCount;
static int clockSyncNextSample;
static bool clockSyncValid;
static int64_t clockSyncOffsetUs;
static uint64_t clockSyncHostUs;
static uint32_t clockSyncSentSequence[CLOCK_SYNC_SENT_SLOTS];
static uint64_t clockSyncSentUs[CLOCK_SYNC_SENT_SLOTS];

// Initialize the video stream
void initializeVideoStream(void) {
    initializeVideoDepacketizer(StreamConfig.packetSize);
    RtpvInitializeQueue(&rtpQueue);
    decryptionCtx = PltCreateCryptoContext();
    receivedDataFromPeer = false;
    firstDataTimeMs = 0;
    receivedFullFrame = false;
    PltCreateMutex(&partialDeadlineLock);
    partialDeadlineEnabled = false;
    partialDeadlineOffsetUs = 0;
    partialDeadlineMinPercent = 0;
    PltCreateMutex(&clockSyncLock);
    clockSyncSampleCount = 0;
    clockSyncNextSample = 0;
    clockSyncValid = false;
    clockSyncOffsetUs = 0;
    clockSyncHostUs = 0;
    memset(clockSyncSentSequence, 0, sizeof(clockSyncSentSequence));
    memset(clockSyncSentUs, 0, sizeof(clockSyncSentUs));
}

// Clean up the video stream
void destroyVideoStream(void) {
    PltDestroyCryptoContext(decryptionCtx);
    destroyVideoDepacketizer();
    RtpvCleanupQueue(&rtpQueue);
    PltDeleteMutex(&partialDeadlineLock);
    PltDeleteMutex(&clockSyncLock);
}

static void sendClockSyncRequest(LC_SOCKADDR* saddr, uint32_t sequence) {
    SS_CLOCK_SYNC_REQUEST request;
    uint64_t nowUs;

    memcpy(request.payload, VideoPingPayload.payload, sizeof(request.payload));
    request.magic = LE32(SS_CLOCK_SYNC_REQUEST_MAGIC);
    request.sequence = LE32(sequence);

    nowUs = PltGetMicroseconds();
    request.clientSendUs = LE64(nowUs);

    PltLockMutex(&clockSyncLock);
    clockSyncSentSequence[sequence % CLOCK_SYNC_SENT_SLOTS] = sequence;
    clockSyncSentUs[sequence % CLOCK_SYNC_SENT_SLOTS] = nowUs;
    PltUnlockMutex(&clockSyncLock);

    sendto(rtpSocket, (char*)&request, sizeof(request), 0, (struct sockaddr*)saddr, AddrLen);
}

static bool isClockSyncAnswered(void) {
    bool valid;

    PltLockMutex(&clockSyncLock);
    valid = clockSyncValid;
    PltUnlockMutex(&clockSyncLock);

    return valid;
}

// receivedUs is when the reply came off the socket
static void handleClockSyncReply(PSS_CLOCK_SYNC_REPLY reply, uint64_t receivedUs) {
    uint32_t sequence = LE32(reply->sequence);
    uint64_t clientSendUs = LE64(reply->clientSendUs);
    uint64_t hostReceiveUs = LE64(reply->hostReceiveUs);
    uint64_t hostSendUs = LE64(reply->hostSendUs);
    int slot = sequence % CLOCK_SYNC_SENT_SLOTS;
    int64_t rttUs, offsetUs;
    bool first;
    int i;

    PltLockMutex(&clockSyncLock);

    // Only an answer to a request of ours, once. An off-path sender can't know the send
    // time it has to echo.
    if (clientSendUs == 0 || clockSyncSentSequence[slot] != sequence || clockSyncSentUs[slot] != clientSendUs) {
        PltUnlockMutex(&clockSyncLock);
        return;
    }
    clockSyncSentUs[slot] = 0;

    rttUs = ((int64_t)receivedUs - (int64_t)clientSendUs) - ((int64_t)hostSendUs - (int64_t)hostReceiveUs);
    if (rttUs < 0 || rttUs > CLOCK_SYNC_MAX_RTT_US || hostSendUs < hostReceiveUs) {
        PltUnlockMutex(&clockSyncLock);
        return;
    }

    // The midpoints of the two legs, assuming they take as long as each other
    offsetUs = (((int64_t)hostReceiveUs - (int64_t)clientSendUs) + ((int64_t)hostSendUs - (int64_t)receivedUs)) / 2;

    clockSyncSamples[clockSyncNextSample].rttUs = rttUs;
    clockSyncSamples[clockSyncNextSample].offsetUs = offsetUs;
    clockSyncSamples[clockSyncNextSample].hostUs = hostReceiveUs;
    clockSyncNextSample = (clockSyncNextSample + 1) % CLOCK_SYNC_SAMPLES;
    if (clockSyncSampleCount < CLOCK_SYNC_SAMPLES) {
        clockSyncSampleCount++;
    }

    {
        int best = 0;
        for (i = 1; i < clockSyncSampleCount; i++) {
            if (clockSyncSamples[i].rttUs < clockSyncSamples[best].rttUs) {
                best = i;
            }
        }
        clockSyncOffsetUs = clockSyncSamples[best].offsetUs;
        clockSyncHostUs = clockSyncSamples[best].hostUs;
    }

    first = !clockSyncValid;
    clockSyncValid = true;

    PltUnlockMutex(&clockSyncLock);

    if (first) {
        Limelog("Clock sync: host answered, round trip %d us\n", (int)rttUs);
    }
}

bool LiGetHostTimeOnLocalClock(uint64_t presentationTimeUs, uint64_t* localTimeUs) {
    int64_t offsetUs;
    uint64_t hostUs;
    double periods;
    int64_t wraps;
    int64_t hostTimeUs;

    PltLockMutex(&clockSyncLock);
    if (!clockSyncValid) {
        PltUnlockMutex(&clockSyncLock);
        return false;
    }
    offsetUs = clockSyncOffsetUs;
    hostUs = clockSyncHostUs;
    PltUnlockMutex(&clockSyncLock);

    // presentationTimeUs wraps with the RTP timestamp, while the host's sync times don't, so
    // take the wrap that puts the frame nearest the latest sync
    periods = ((double)hostUs - (double)presentationTimeUs) / RTP_TIMESTAMP_PERIOD_US;
    wraps = periods >= 0 ? (int64_t)(periods + 0.5) : -(int64_t)(0.5 - periods);
    hostTimeUs = (int64_t)presentationTimeUs + (int64_t)((double)wraps * RTP_TIMESTAMP_PERIOD_US);

    if (hostTimeUs - offsetUs < 0) {
        return false;
    }

    *localTimeUs = (uint64_t)(hostTimeUs - offsetUs);
    return true;
}

void LiSetPartialFrameDeadline(bool enabled, int64_t offsetUs, int minReceivedPercent) {
    PltLockMutex(&partialDeadlineLock);
    partialDeadlineEnabled = enabled;
    partialDeadlineOffsetUs = offsetUs;
    partialDeadlineMinPercent = minReceivedPercent;
    PltUnlockMutex(&partialDeadlineLock);
}

bool getPartialFrameDeadline(int64_t* offsetUs, int* minReceivedPercent) {
    bool enabled;

    PltLockMutex(&partialDeadlineLock);
    enabled = partialDeadlineEnabled;
    *offsetUs = partialDeadlineOffsetUs;
    *minReceivedPercent = partialDeadlineMinPercent;
    PltUnlockMutex(&partialDeadlineLock);

    return enabled;
}

// Cuts the current frame short if it has reached its deadline. Returns how long until the
// deadline of the frame still arriving, or -1 if there is none.
static int64_t checkPartialFrameDeadline(void) {
    uint64_t deadlineUs, nowUs;

    if (!RtpvGetPartialFrameDeadline(&rtpQueue, &deadlineUs)) {
        return -1;
    }

    nowUs = PltGetMicroseconds();
    RtpvCheckPartialFrameDeadline(&rtpQueue, nowUs);

    if (RtpvGetPartialFrameDeadline(&rtpQueue, &deadlineUs) && nowUs < deadlineUs) {
        return (int64_t)(deadlineUs - nowUs);
    }
    return -1;
}

// UDP Ping proc
static void VideoPingThreadProc(void* context) {
    char legacyPingData[] = { 0x50, 0x49, 0x4E, 0x47 };
    LC_SOCKADDR saddr;

    LC_ASSERT(VideoPortNumber != 0);

    memcpy(&saddr, &RemoteAddr, sizeof(saddr));
    SET_PORT(&saddr, VideoPortNumber);

    // We do not check for errors here. Socket errors will be handled
    // on the read-side in ReceiveThreadProc(). This avoids potential
    // issues related to receiving ICMP port unreachable messages due
    // to sending a packet prior to the host PC binding to that port.
    int pingCount = 0;
    bool pinged = false;
    uint64_t lastPingMs = 0;
    uint32_t clockSyncSequence = 0;
    uint64_t clockSyncStartMs = 0;
    bool clockSyncGaveUp = false;
    while (!PltIsThreadInterrupted(&udpPingThread)) {
        uint64_t nowMs = PltGetMillis();
        bool clockSyncing;

        if (!pinged || nowMs - lastPingMs >= 500) {
            if (VideoPingPayload.payload[0] != 0) {
                pingCount++;
                VideoPingPayload.sequenceNumber = BE32(pingCount);

                sendto(rtpSocket, (char*)&VideoPingPayload, sizeof(VideoPingPayload), 0, (struct sockaddr*)&saddr, AddrLen);
            }
            else {
                sendto(rtpSocket, legacyPingData, sizeof(legacyPingData), 0, (struct sockaddr*)&saddr, AddrLen);
            }

            pinged = true;
            lastPingMs = nowMs;
        }

        // Clock sync starts once video arrives, by when the host knows where to answer.
        // Hosts other than this fork's Sunshine never answer, and then it stops.
        clockSyncing = IS_SUNSHINE() && VideoPingPayload.payload[0] != 0 && !clockSyncGaveUp;
        if (clockSyncing && receivedDataFromPeer) {
            if (clockSyncStartMs == 0) {
                clockSyncStartMs = nowMs;
            }

            if (!isClockSyncAnswered() && nowMs - clockSyncStartMs >= CLOCK_SYNC_GIVE_UP_MS) {
                Limelog("Clock sync: no answer from the host, so it is not supported\n");
                clockSyncGaveUp = true;
                clockSyncing = false;
            }
            else {
                sendClockSyncRequest(&saddr, ++clockSyncSequence);
            }
        }

        PltSleepMsInterruptible(&udpPingThread, clockSyncing ? CLOCK_SYNC_INTERVAL_MS : 500);
    }
}

// Receive thread proc
static void VideoReceiveThreadProc(void* context) {
    int err;
    int bufferSize, receiveSize, decryptedSize, minSize;
    char* buffer;
    char* encryptedBuffer;
    int queueStatus;
    bool useSelect;
    int waitingForVideoMs;
    bool encrypted;

    encrypted = !!(EncryptionFeaturesEnabled & SS_ENC_VIDEO);
    decryptedSize = StreamConfig.packetSize + MAX_RTP_HEADER_SIZE;
    minSize = sizeof(RTP_PACKET) + ((EncryptionFeaturesEnabled & SS_ENC_VIDEO) ? sizeof(ENC_VIDEO_HEADER) : 0);
    receiveSize = decryptedSize + ((EncryptionFeaturesEnabled & SS_ENC_VIDEO) ? sizeof(ENC_VIDEO_HEADER) : 0);
    bufferSize = decryptedSize + sizeof(RTPV_QUEUE_ENTRY);
    buffer = NULL;

    if (setNonFatalRecvTimeoutMs(rtpSocket, UDP_RECV_POLL_TIMEOUT_MS) < 0) {
        // SO_RCVTIMEO failed, so use select() to wait
        useSelect = true;
    }
    else {
        // SO_RCVTIMEO timeout set for recv()
        useSelect = false;
    }

    // Allocate a staging buffer to use for each received packet
    if (encrypted) {
        encryptedBuffer = (char*)malloc(receiveSize);
        if (encryptedBuffer == NULL) {
            Limelog("Video Receive: malloc() failed\n");
            ListenerCallbacks.connectionTerminated(-1);
            return;
        }
    }
    else {
        encryptedBuffer = NULL;
    }

    waitingForVideoMs = 0;
    while (!PltIsThreadInterrupted(&receiveThread)) {
        PRTP_PACKET packet;

        if (buffer == NULL) {
            buffer = (char*)malloc(bufferSize);
            if (buffer == NULL) {
                Limelog("Video Receive: malloc() failed\n");
                ListenerCallbacks.connectionTerminated(-1);
                break;
            }
        }

        // Cut the current frame short if it has reached its deadline, and wake up for the
        // deadline even if no more packets come. A packet already waiting is taken without
        // blocking, which costs no more than an ordinary receive; only when none is there does
        // the thread wait, in poll(), until the deadline. On Windows, where a receive can't be
        // made non-blocking per call, it waits in select() once the deadline is close; a recv()
        // with a short SO_RCVTIMEO there often went 10 ms without returning.
        int64_t untilDeadlineUs = checkPartialFrameDeadline();
        bool received = false;
        if (untilDeadlineUs >= 0) {
            struct pollfd pfd;
            int timeoutMs = (int)((untilDeadlineUs + 999) / 1000);
            if (timeoutMs > UDP_RECV_POLL_TIMEOUT_MS) {
                timeoutMs = UDP_RECV_POLL_TIMEOUT_MS;
            }

            pfd.fd = rtpSocket;
            pfd.events = POLLIN;
#ifdef MSG_DONTWAIT
            err = (int)recvfrom(rtpSocket, encrypted ? encryptedBuffer : buffer, receiveSize, MSG_DONTWAIT, NULL, NULL);
            if (err >= 0) {
                received = true;
            }
            else if (LastSocketError() == EWOULDBLOCK || LastSocketError() == EAGAIN || LastSocketError() == EINTR) {
                if (pollSockets(&pfd, 1, timeoutMs) == 0) {
                    // Nothing arrived before the deadline
                    continue;
                }
            }
#else
            if (untilDeadlineUs <= PARTIAL_DEADLINE_POLL_WINDOW_US && pollSockets(&pfd, 1, timeoutMs) == 0) {
                // Nothing arrived before the deadline
                continue;
            }
#endif
        }

        if (!received) {
            err = recvUdpSocket(rtpSocket,
                                encrypted ? encryptedBuffer : buffer,
                                receiveSize,
                                useSelect);
        }
        if (err < 0) {
            Limelog("Video Receive: recvUdpSocket() failed: %d\n", (int)LastSocketError());
            ListenerCallbacks.connectionTerminated(LastSocketFail());
            break;
        }
        else if  (err == 0) {
            if (!receivedDataFromPeer) {
                // If we wait many seconds without ever receiving a video packet,
                // assume something is broken and terminate the connection.
                waitingForVideoMs += UDP_RECV_POLL_TIMEOUT_MS;
                if (waitingForVideoMs >= FIRST_FRAME_TIMEOUT_SEC * 1000) {
                    Limelog("Terminating connection due to lack of video traffic\n");
                    ListenerCallbacks.connectionTerminated(ML_ERROR_NO_VIDEO_TRAFFIC);
                    break;
                }
            }

            // Receive timed out; try again
            continue;
        }

        // A clock sync reply, which can't be a video packet (see SS_CLOCK_SYNC_REPLY)
        if (err == sizeof(SS_CLOCK_SYNC_REPLY)) {
            uint64_t receivedUs = PltGetMicroseconds();
            PSS_CLOCK_SYNC_REPLY reply = (PSS_CLOCK_SYNC_REPLY)(encrypted ? encryptedBuffer : buffer);

            if (LE32(reply->magic) == SS_CLOCK_SYNC_REPLY_MAGIC) {
                handleClockSyncReply(reply, receivedUs);
                continue;
            }
        }

        // Every packet counts, including FEC, duplicates, and packets of frames that are
        // later dropped
        rtpQueue.stats.byteCountReceived += (uint32_t)err;
        rtpQueue.stats.packetCountReceived++;

        if (!receivedDataFromPeer) {
            receivedDataFromPeer = true;
            Limelog("Received first video packet after %d ms\n", waitingForVideoMs);

            firstDataTimeMs = PltGetMillis();
        }

#ifndef LC_FUZZING
        if (!receivedFullFrame) {
            if (PltGetMillis() - firstDataTimeMs >= FIRST_FRAME_TIMEOUT_SEC * 1000) {
                Limelog("Terminating connection due to lack of a successful video frame\n");
                ListenerCallbacks.connectionTerminated(ML_ERROR_NO_VIDEO_FRAME);
                break;
            }
        }
#endif

        if (err < minSize) {
            // Runt packet
            continue;
        }

        // Decrypt the packet into the buffer if encryption is enabled
        if (encrypted) {
            PENC_VIDEO_HEADER encHeader = (PENC_VIDEO_HEADER)encryptedBuffer;

            // If this frame is below our current frame number, discard it before decryption
            // to save CPU cycles decrypting FEC shards for a frame we already reassembled.
            //
            // Since this is happening _before_ decryption, this packet is not trusted yet.
            // It's imperative that we do not mutate any state based on this packet until
            // after it has been decrypted successfully!
            //
            // It's possible for an attacker to inject a fake packet that has any value of
            // header fields they want, however this provides them no benefit because we will
            // simply drop said packet here (if it's below the current frame number) or it
            // will pass this check and be dropped during decryption (if contents is tampered)
            // or after decryption in the RTP queue (if it's a replay of a previous authentic
            // packet from the host).
            //
            // In short, an attacker spoofing this value via MITM or sending malicious values
            // impersonating the host from off-link doesn't gain them anything. If they have
            // a true MITM, they can DoS our connection by just dropping all our traffic, so
            // tampering with packets to fail this check doesn't accomplish anything they
            // couldn't already do. If they're not on-link, we just throw their malicious
            // traffic away (as mentioned in the paragraph above) and continue accepting
            // legitmate video traffic.
            if (encHeader->frameNumber && LE32(encHeader->frameNumber) < RtpvGetCurrentFrameNumber(&rtpQueue)) {
                continue;
            }

            if (!PltDecryptMessage(decryptionCtx, ALGORITHM_AES_GCM, 0,
                                   (unsigned char*)StreamConfig.remoteInputAesKey, sizeof(StreamConfig.remoteInputAesKey),
                                   encHeader->iv, sizeof(encHeader->iv),
                                   encHeader->tag, sizeof(encHeader->tag),
                                   ((unsigned char*)(encHeader + 1)), err - sizeof(ENC_VIDEO_HEADER), // The ciphertext is after the header
                                   (unsigned char*)buffer, &err)) {
                Limelog("Failed to decrypt video packet!\n");
                continue;
            }
        }

        // Convert fields to host byte-order
        packet = (PRTP_PACKET)&buffer[0];
        packet->sequenceNumber = BE16(packet->sequenceNumber);
        packet->timestamp = BE32(packet->timestamp);
        packet->ssrc = BE32(packet->ssrc);

        queueStatus = RtpvAddPacket(&rtpQueue, packet, err, (PRTPV_QUEUE_ENTRY)&buffer[decryptedSize]);

        if (queueStatus == RTPF_RET_QUEUED) {
            // The queue owns the buffer
            buffer = NULL;
        }
    }

    if (buffer != NULL) {
        free(buffer);
    }

    if (encryptedBuffer != NULL) {
        free(encryptedBuffer);
    }
}

void notifyKeyFrameReceived(void) {
    // Remember that we got a full frame successfully
    receivedFullFrame = true;
}

// Decoder thread proc
static void VideoDecoderThreadProc(void* context) {
    while (!PltIsThreadInterrupted(&decoderThread)) {
        VIDEO_FRAME_HANDLE frameHandle;
        PDECODE_UNIT decodeUnit;

        if (!LiWaitForNextVideoFrame(&frameHandle, &decodeUnit)) {
            return;
        }

        LiCompleteVideoFrame(frameHandle, VideoCallbacks.submitDecodeUnit(decodeUnit));
    }
}

// Read the first frame of the video stream
int readFirstFrame(void) {
    // All that matters is that we close this socket.
    // This starts the flow of video on Gen 3 servers.

    closeSocket(firstFrameSocket);
    firstFrameSocket = INVALID_SOCKET;

    return 0;
}

// Terminate the video stream
void stopVideoStream(void) {
    if (!receivedDataFromPeer) {
        Limelog("No video traffic was ever received from the host!\n");
    }

    VideoCallbacks.stop();

    // Wake up client code that may be waiting on the decode unit queue
    stopVideoDepacketizer();

    PltInterruptThread(&udpPingThread);
    PltInterruptThread(&receiveThread);
    if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
        PltInterruptThread(&decoderThread);
    }

    if (firstFrameSocket != INVALID_SOCKET) {
        shutdownTcpSocket(firstFrameSocket);
    }

    PltJoinThread(&udpPingThread);
    PltJoinThread(&receiveThread);
    if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
        PltJoinThread(&decoderThread);
    }

    if (firstFrameSocket != INVALID_SOCKET) {
        closeSocket(firstFrameSocket);
        firstFrameSocket = INVALID_SOCKET;
    }
    if (rtpSocket != INVALID_SOCKET) {
        closeSocket(rtpSocket);
        rtpSocket = INVALID_SOCKET;
    }

    VideoCallbacks.cleanup();
}

// Start the video stream
int startVideoStream(void* rendererContext, int drFlags) {
    int err;

    firstFrameSocket = INVALID_SOCKET;

    // This must be called before the decoder thread starts submitting
    // decode units
    LC_ASSERT(NegotiatedVideoFormat != 0);
    err = VideoCallbacks.setup(NegotiatedVideoFormat, StreamConfig.width,
        StreamConfig.height, StreamConfig.fps, rendererContext, drFlags);
    if (err != 0) {
        return err;
    }

    rtpSocket = bindUdpSocket(RemoteAddr.ss_family, &LocalAddr, AddrLen,
                              RTP_RECV_PACKETS_BUFFERED * (StreamConfig.packetSize + MAX_RTP_HEADER_SIZE),
                              SOCK_QOS_TYPE_VIDEO);
    if (rtpSocket == INVALID_SOCKET) {
        VideoCallbacks.cleanup();
        return LastSocketError();
    }

    VideoCallbacks.start();

    err = PltCreateThread("VideoRecv", VideoReceiveThreadProc, NULL, &receiveThread);
    if (err != 0) {
        VideoCallbacks.stop();
        closeSocket(rtpSocket);
        VideoCallbacks.cleanup();
        return err;
    }

    if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
        err = PltCreateThread("VideoDec", VideoDecoderThreadProc, NULL, &decoderThread);
        if (err != 0) {
            VideoCallbacks.stop();
            PltInterruptThread(&receiveThread);
            PltJoinThread(&receiveThread);
            closeSocket(rtpSocket);
            VideoCallbacks.cleanup();
            return err;
        }
    }

    if (AppVersionQuad[0] == 3) {
        // Connect this socket to open port 47998 for our ping thread
        firstFrameSocket = connectTcpSocket(&RemoteAddr, AddrLen,
                                            FIRST_FRAME_PORT, FIRST_FRAME_TIMEOUT_SEC);
        if (firstFrameSocket == INVALID_SOCKET) {
            VideoCallbacks.stop();
            stopVideoDepacketizer();
            PltInterruptThread(&receiveThread);
            if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
                PltInterruptThread(&decoderThread);
            }
            PltJoinThread(&receiveThread);
            if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
                PltJoinThread(&decoderThread);
            }
            closeSocket(rtpSocket);
            VideoCallbacks.cleanup();
            return LastSocketError();
        }
    }

    // Start pinging before reading the first frame so GFE knows where
    // to send UDP data
    err = PltCreateThread("VideoPing", VideoPingThreadProc, NULL, &udpPingThread);
    if (err != 0) {
        VideoCallbacks.stop();
        stopVideoDepacketizer();
        PltInterruptThread(&receiveThread);
        if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
            PltInterruptThread(&decoderThread);
        }
        PltJoinThread(&receiveThread);
        if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
            PltJoinThread(&decoderThread);
        }
        closeSocket(rtpSocket);
        if (firstFrameSocket != INVALID_SOCKET) {
            closeSocket(firstFrameSocket);
            firstFrameSocket = INVALID_SOCKET;
        }
        VideoCallbacks.cleanup();
        return err;
    }

    if (AppVersionQuad[0] == 3) {
        // Read the first frame to start the flow of video
        err = readFirstFrame();
        if (err != 0) {
            stopVideoStream();
            return err;
        }
    }

    return 0;
}

const RTP_VIDEO_STATS* LiGetRTPVideoStats(void) {
    return &rtpQueue.stats;
}
