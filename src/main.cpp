#include <Arduino.h>
#include "uwb_tdma.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    uwb_init();
}

void loop()
{
    uwb_loop();
}