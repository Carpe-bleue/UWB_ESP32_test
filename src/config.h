#pragma once

#ifndef NODE_ID
#define NODE_ID 1   // default when build_flags does not provide NODE_ID
#endif

#define NUM_NODES 4

// Bring-up mode: one node initiates ranging, all others only respond.
#define INITIATOR_NODE_ID 1

// Stable debug mode: initiator ranges only to one target with low traffic.
#define SINGLE_PAIR_DEBUG_MODE 1
#define DEBUG_TARGET_NODE_ID 2

// slot duration (ms)
#define SLOT_TIME_MS 30

// Delay between initiator ranging attempts in bring-up mode.
#define RANGING_PERIOD_MS 120

// Lower traffic and add retries for stability during bring-up.
#define RANGING_PERIOD_DEBUG_MS 800
#define MAX_INITIATOR_RETRIES 3

// TDMA pairs
struct Pair {
    uint8_t a;
    uint8_t b;
};

static const Pair schedule[] = {
    {1,2},
    {1,3},
    {1,4},
    {2,3},
    {2,4},
    {3,4}
};

#define NUM_SLOTS (sizeof(schedule)/sizeof(schedule[0]))