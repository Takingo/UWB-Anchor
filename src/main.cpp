/*
 * DS-TWR Test - SPI Debug
 */

#include "dw3000.h"
#include "SPI.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== DW3000 SPI Test ===");

    // Initialize DW3000 library
    if (DW3000.begin(PIN_RST, PIN_SS, PIN_IRQ)) {
        Serial.println("DW3000 initialization successful.");
        // Example: Start as Anchor (replace with actual logic)
        // DW3000.startAsAnchor("A1", MODE_LONGDATA_RANGE_ACCURACY);
    } else {
        Serial.println("DW3000 initialization failed.");
    }
}

void loop()
{
    delay(1000);
}