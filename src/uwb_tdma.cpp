#include "uwb_tdma.h"

static_assert(NODE_ID >= 1 && NODE_ID <= NUM_NODES, "NODE_ID must be within 1..NUM_NODES");
static_assert(INITIATOR_NODE_ID >= 1 && INITIATOR_NODE_ID <= NUM_NODES, "INITIATOR_NODE_ID must be within 1..NUM_NODES");
static_assert(DEBUG_TARGET_NODE_ID >= 1 && DEBUG_TARGET_NODE_ID <= NUM_NODES, "DEBUG_TARGET_NODE_ID must be within 1..NUM_NODES");
static_assert(DEBUG_TARGET_NODE_ID != INITIATOR_NODE_ID, "DEBUG_TARGET_NODE_ID must differ from INITIATOR_NODE_ID");

static const uint32_t RESPONSE_TIMEOUT_MS = 120;
static const uint32_t RESPONDER_STAGE_TIMEOUT_MS = 250;

static uint8_t nextTarget = 2;
static uint8_t responderStage = 0;
static uint8_t responderPeer = 0;
static uint32_t responderStageStartedMs = 0;
static unsigned long long responderRxTime = 0;
static unsigned long long responderTxTime = 0;
static int responderReplyTime = 0;

static int waitForReceivedFrame(uint32_t timeoutMs)
{
    const uint32_t started = millis();
    while (millis() - started < timeoutMs) {
        const int rxStatus = DW3000.receivedFrameSucc();
        if (rxStatus != 0) {
            return rxStatus;
        }
        delay(1);
    }
    return 0;
}

static bool receivedFrameIsForThisNode(uint8_t expectedSender, uint8_t expectedStage)
{
    const uint8_t sender = DW3000.getSenderID();
    const uint8_t destination = DW3000.getDestinationID();
    const uint8_t stage = DW3000.ds_getStage();

    if (DW3000.ds_isErrorFrame()) {
        Serial.println("[WARN] Received DS-TWR error frame");
        return false;
    }

    if (sender != expectedSender || destination != NODE_ID || stage != expectedStage) {
        Serial.printf(
            "[WARN] Ignoring frame: sender=%u dest=%u stage=%u, expected sender=%u dest=%u stage=%u\n",
            sender,
            destination,
            stage,
            expectedSender,
            NODE_ID,
            expectedStage
        );
        return false;
    }

    return true;
}

static void sendRangingInfo(uint8_t peerId, int roundTime, int replyTime)
{
    DW3000.setMode(1);
    DW3000.write(0x14, 0x01, NODE_ID & 0xFF);
    DW3000.write(0x14, 0x02, peerId & 0xFF);
    DW3000.write(0x14, 0x03, 4);
    DW3000.write(0x14, 0x04, static_cast<uint32_t>(roundTime));
    DW3000.write(0x14, 0x08, static_cast<uint32_t>(replyTime));
    DW3000.setFrameLength(12);
    DW3000.TXInstantRX();
}

static void resetResponderState()
{
    responderStage = 0;
    responderPeer = 0;
    responderStageStartedMs = 0;
    responderRxTime = 0;
    responderTxTime = 0;
    responderReplyTime = 0;
    DW3000.standardRX();
}

void uwb_init()
{
    DW3000.begin();

    DW3000.hardReset();
    delay(200);

    if (!DW3000.checkSPI()) {
        Serial.println("[ERROR] SPI communication failed!");
        while (1);
    }

    while (!DW3000.checkForIDLE()) {
        Serial.println("[ERROR] IDLE1 FAILED");
        delay(1000);
    }

    DW3000.softReset();
    delay(200);

    while (!DW3000.checkForIDLE()) {
        Serial.println("[ERROR] IDLE2 FAILED");
        delay(1000);
    }

    DW3000.init();
    Serial.println("[INFO] DW3000.init() completed");

    DW3000.configureAsTX();
    DW3000.setupGPIO();

    DW3000.setTXAntennaDelay(ANT_DELAY);
    DW3000.setSenderID(NODE_ID);

    DW3000.clearSystemStatus();
    if (NODE_ID != INITIATOR_NODE_ID) {
        DW3000.standardRX();
    }

    Serial.printf("[INFO] DW3000 initialized (NODE_ID=%d)\n", NODE_ID);
    if (NODE_ID == INITIATOR_NODE_ID) {
        Serial.println("[INFO] Role: initiator");
    } else {
        Serial.println("[INFO] Role: responder");
    }
}

static bool doInitiator(uint8_t targetId)
{
    int rxStatus = 0;
    int tRoundA = 0;
    int tReplyA = 0;

    Serial.printf("[RANGE] Node %u -> Node %u\n", NODE_ID, targetId);

    DW3000.setDestinationID(targetId);
    DW3000.clearSystemStatus();
    DW3000.ds_sendFrame(1);
    const unsigned long long txPollTime = DW3000.readTXTimestamp();

    rxStatus = waitForReceivedFrame(RESPONSE_TIMEOUT_MS);
    if (rxStatus != 1) {
        Serial.printf("[WARN] No valid response from Node %u (rxStatus=%d)\n", targetId, rxStatus);
        DW3000.clearSystemStatus();
        return false;
    }

    const unsigned long long rxResponseTime = DW3000.readRXTimestamp();
    if (!receivedFrameIsForThisNode(targetId, 2)) {
        DW3000.clearSystemStatus();
        return false;
    }
    DW3000.clearSystemStatus();

    DW3000.setDestinationID(targetId);
    DW3000.ds_sendFrame(3);
    tRoundA = static_cast<int>(rxResponseTime - txPollTime);

    const unsigned long long txFinalTime = DW3000.readTXTimestamp();
    tReplyA = static_cast<int>(txFinalTime - rxResponseTime);

    rxStatus = waitForReceivedFrame(RESPONSE_TIMEOUT_MS);
    if (rxStatus != 1) {
        Serial.printf("[WARN] No ranging-info frame from Node %u (rxStatus=%d)\n", targetId, rxStatus);
        DW3000.clearSystemStatus();
        return false;
    }

    if (!receivedFrameIsForThisNode(targetId, 4)) {
        DW3000.clearSystemStatus();
        return false;
    }

    const int clockOffset = DW3000.getRawClockOffset();
    const int tRoundB = static_cast<int>(DW3000.read(0x12, 0x04));
    const int tReplyB = static_cast<int>(DW3000.read(0x12, 0x08));
    const int rangingTime = DW3000.ds_processRTInfo(tRoundA, tReplyA, tRoundB, tReplyB, clockOffset);
    const double distanceCm = DW3000.convertToCM(rangingTime);

    Serial.printf("[DIST] Node %u -> Node %u: ", NODE_ID, targetId);
    DW3000.printDouble(distanceCm, 100, false);
    Serial.println(" cm");

    DW3000.clearSystemStatus();
    return true;
}

static void doResponder()
{
    if (SINGLE_PAIR_DEBUG_MODE && NODE_ID != DEBUG_TARGET_NODE_ID) {
        return;
    }

    if (responderStage == 2 && millis() - responderStageStartedMs > RESPONDER_STAGE_TIMEOUT_MS) {
        Serial.printf("[WARN] Responder timeout waiting for final frame from Node %u\n", responderPeer);
        DW3000.clearSystemStatus();
        resetResponderState();
        return;
    }

    const int rxStatus = DW3000.receivedFrameSucc();
    if (rxStatus == 0) {
        return;
    }

    if (rxStatus != 1) {
        Serial.printf("[WARN] Receiver error on Node %u (rxStatus=%d)\n", NODE_ID, rxStatus);
        DW3000.clearSystemStatus();
        resetResponderState();
        return;
    }

    const uint8_t sender = DW3000.getSenderID();
    const uint8_t destination = DW3000.getDestinationID();
    const uint8_t stage = DW3000.ds_getStage();

    if (DW3000.ds_isErrorFrame()) {
        Serial.println("[WARN] Received DS-TWR error frame");
        DW3000.clearSystemStatus();
        resetResponderState();
        return;
    }

    if (destination != NODE_ID) {
        DW3000.clearSystemStatus();
        DW3000.standardRX();
        return;
    }

    if (SINGLE_PAIR_DEBUG_MODE && sender != INITIATOR_NODE_ID) {
        DW3000.clearSystemStatus();
        DW3000.standardRX();
        return;
    }

    if (responderStage == 0) {
        if (stage != 1) {
            Serial.printf("[WARN] Unexpected first responder stage=%u from Node %u\n", stage, sender);
            DW3000.clearSystemStatus();
            DW3000.standardRX();
            return;
        }

        responderPeer = sender;
        responderRxTime = DW3000.readRXTimestamp();
        DW3000.clearSystemStatus();

        DW3000.setDestinationID(responderPeer);
        DW3000.ds_sendFrame(2);
        responderTxTime = DW3000.readTXTimestamp();
        responderReplyTime = static_cast<int>(responderTxTime - responderRxTime);
        responderStage = 2;
        responderStageStartedMs = millis();

        Serial.printf("[RX] Poll from Node %u, response sent\n", responderPeer);
        return;
    }

    if (responderStage == 2) {
        if (sender != responderPeer || stage != 3) {
            Serial.printf(
                "[WARN] Unexpected final frame: sender=%u stage=%u, expected sender=%u stage=3\n",
                sender,
                stage,
                responderPeer
            );
            DW3000.clearSystemStatus();
            resetResponderState();
            return;
        }

        const unsigned long long rxFinalTime = DW3000.readRXTimestamp();
        const int responderRoundTime = static_cast<int>(rxFinalTime - responderTxTime);
        DW3000.clearSystemStatus();

        sendRangingInfo(responderPeer, responderRoundTime, responderReplyTime);
        Serial.printf("[TX] Ranging info sent to Node %u\n", responderPeer);

        responderStage = 0;
        responderPeer = 0;
        responderStageStartedMs = 0;
        return;
    }

    DW3000.clearSystemStatus();
    resetResponderState();
}

void uwb_loop()
{
    static uint32_t lastRangeTimeMs = 0;

    if (NODE_ID != INITIATOR_NODE_ID) {
        doResponder();
        return;
    }

    const uint32_t periodMs = SINGLE_PAIR_DEBUG_MODE ? RANGING_PERIOD_DEBUG_MS : RANGING_PERIOD_MS;
    if (millis() - lastRangeTimeMs < periodMs) {
        return;
    }
    lastRangeTimeMs = millis();

    uint8_t targetId = SINGLE_PAIR_DEBUG_MODE ? DEBUG_TARGET_NODE_ID : nextTarget;
    if (targetId == NODE_ID || targetId > NUM_NODES) {
        targetId = 1;
        if (targetId == NODE_ID) {
            targetId++;
        }
    }

    bool ok = false;
    for (int attempt = 0; attempt < MAX_INITIATOR_RETRIES; attempt++) {
        ok = doInitiator(targetId);
        if (ok) {
            break;
        }
        delay(20);
    }

    if (!SINGLE_PAIR_DEBUG_MODE) {
        nextTarget = targetId + 1;
    }

    if (!ok) {
        Serial.printf("[WARN] Node %u: ranging retries exhausted for Node %u\n", NODE_ID, targetId);
    }
}
