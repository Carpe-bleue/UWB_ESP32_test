#ifndef UWB_TDMA_H
#define UWB_TDMA_H

#include <DW3000.h>
#include "config.h"

// default antenna delay calibration
#define ANT_DELAY 16350

// Initialize DW3000 and UWB settings
void uwb_init();

// Main loop: handle TDMA slot and DS-TWR exchange
void uwb_loop();

#endif
