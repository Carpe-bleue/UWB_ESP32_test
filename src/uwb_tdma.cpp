#include "uwb_tdma.h"

static_assert(NODE_ID >= 1 && NODE_ID <= NUM_NODES, "NODE_ID must be within 1..NUM_NODES");
static_assert(INITIATOR_NODE_ID >= 1 && INITIATOR_NODE_ID <= NUM_NODES, "INITIATOR_NODE_ID must be within 1..NUM_NODES");
static_assert(DEBUG_TARGET_NODE_ID >= 1 && DEBUG_TARGET_NODE_ID <= NUM_NODES, "DEBUG_TARGET_NODE_ID must be within 1..NUM_NODES");
static_assert(DEBUG_TARGET_NODE_ID != INITIATOR_NODE_ID, "DEBUG_TARGET_NODE_ID must differ from INITIATOR_NODE_ID");

// Variables to store timestamps
static uint64_t txInitTime, rxRespTime, txFinalTime;
static uint64_t respRxTime, respTxTime;

static const uint32_t PING_BASE = 1000;

// Initialization: configure DW3000 chip and parameters// Initialization: configure DW3000 chip and parameters
void uwb_init()
{
    DW3000.begin();

    DW3000.hardReset();
    delay(200);

    if (!DW3000.checkSPI())
    {
        Serial.println("[ERROR] SPI communication failed!");
        while (1);
    }

    while (!DW3000.checkForIDLE())
    {
        Serial.println("[ERROR] IDLE1 FAILED");
        delay(1000);
    }

    DW3000.softReset();
    delay(200);

    while (!DW3000.checkForIDLE())
    {
        Serial.println("[ERROR] IDLE2 FAILED");
        delay(1000);
    }

    DW3000.init();                    // <-- void, no if(!)
    Serial.println("[INFO] DW3000.init() completed");

    DW3000.configureAsTX();
    DW3000.setupGPIO();

    DW3000.setTXAntennaDelay(ANT_DELAY);
    DW3000.setSenderID(NODE_ID);

    DW3000.clearSystemStatus();
    DW3000.standardRX();

    Serial.printf("[INFO] DW3000 initialized (NODE_ID=%d)\n", NODE_ID);
    if (NODE_ID == INITIATOR_NODE_ID) {
        Serial.println("[INFO] Role: initiator");
    } else {
        Serial.println("[INFO] Role: responder");
    }
}

// Attempt one DS-TWR exchange: initiator -> targetId
bool doInitiator(uint8_t targetId) {
    static uint32_t pingCounter = 0;
    uint32_t pingFrame = PING_BASE + pingCounter;

    DW3000.clearSystemStatus();
    
    // Critical: Set destination before TX
    DW3000.setDestinationID(targetId);
    DW3000.setTXFrame(pingFrame);
    DW3000.setFrameLength(9);   // Make sure this matches payload size
    
    DW3000.standardTX();

    uint32_t txStart = micros();
    bool success = false;
    
    while (micros() - txStart < 50000) {  // 50ms timeout
        if (DW3000.sentFrameSucc()) {
            success = true;
            break;
        }
        delayMicroseconds(100);  // Small yield
    }

    if (success) {
        Serial.printf("[TX OK] Ping %u to Node %d\n", pingFrame, targetId);
        pingCounter++;
        
        // Switch back to RX
        DW3000.clearSystemStatus();
        DW3000.standardRX();
        return true;
    } else {
        Serial.println("[ERROR] TX timeout (PING)");
        Serial.printf("  Final Status: 0x%08X\n", DW3000.read(0x00, 0x44));  // SYS_STATUS
        DW3000.clearSystemStatus();
        DW3000.standardRX();
        return false;
    }
}
// Responder: if a poll arrives, reply with pong; if final arrives, do nothing extra.
void doResponder() {
    // In debug mode, only the target responds (for low traffic). Otherwise all listen.
    if (SINGLE_PAIR_DEBUG_MODE && NODE_ID != DEBUG_TARGET_NODE_ID && NODE_ID != INITIATOR_NODE_ID) {
        return;  // Only initiator + debug target active
    }

    DW3000.standardRX();
    uint32_t start = micros();
    while (micros() - start < 60000) {
        int rxStatus = DW3000.receivedFrameSucc();
        if (rxStatus == 1) {
            uint32_t rxFrame = DW3000.read(0x12, 0x00);
            DW3000.clearSystemStatus();

            uint32_t txFrame = rxFrame + 1;
            DW3000.setTXFrame(txFrame);
            DW3000.setFrameLength(9);
            DW3000.standardTX();

            uint32_t txStart = micros();
            while (!(DW3000.sentFrameSucc())) {
                if (micros() - txStart > 20000) {
                    Serial.println("[ERROR] TX timeout (PONG)");
                    DW3000.clearSystemStatus();
                    DW3000.standardRX();
                    return;
                }
            }

            DW3000.clearSystemStatus();
            DW3000.standardRX();
            return;
        } else if (rxStatus == 2) {
            DW3000.clearSystemStatus();
        }
    }
}

// Main loop: decide initiator vs responder based on TDMA slot
void uwb_loop() {
    static uint32_t lastRangeTime = millis();
    static uint8_t nextTarget = 2;
    uint32_t now = millis();

    // Keep listening almost all the time.
    doResponder();

    const uint32_t periodMs = SINGLE_PAIR_DEBUG_MODE ? RANGING_PERIOD_DEBUG_MS : RANGING_PERIOD_MS;
    if (NODE_ID == INITIATOR_NODE_ID && now - lastRangeTime >= periodMs) {
        lastRangeTime = now;

        uint8_t targetId = nextTarget;

        if (SINGLE_PAIR_DEBUG_MODE) {
            targetId = DEBUG_TARGET_NODE_ID;
        } else {
            if (targetId == NODE_ID || targetId > NUM_NODES) {
                targetId = 1;
                if (targetId == NODE_ID) {
                    targetId++;
                }
            }
        }

        bool ok = false;
        for (int i = 0; i < MAX_INITIATOR_RETRIES; i++) {
            ok = doInitiator(targetId);
            if (ok) {
                break;
            }
            delay(10);
        }

        if (!SINGLE_PAIR_DEBUG_MODE) {
            nextTarget = targetId + 1;
        }

        if (!ok) {
            Serial.printf("[WARN] Node %d: retries exhausted for Node %d\n", NODE_ID, targetId);
        }
    }
}
