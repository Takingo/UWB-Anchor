/*
 * UWB Anchor Example
 * Uses the refactored DW3000 library
 */

#include "dw3000.h"
#include "SPI.h"

// Pin definitions for ESP32 with Waveshare DW3000 module
#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

// Function to be called when a new range is calculated
void newRangeCallback() {
    Serial.printf("New Range: %.2f m, RSSI: %.2f dBm\n", DW3000.getDistantDeviceRange(), DW3000.getDistantDeviceRSSI());
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== DW3000 UWB Anchor Test ===");

    // Initialize DW3000 library
    if (DW3000.begin(PIN_RST, PIN_SS, PIN_IRQ)) {
        Serial.println("DW3000 initialization successful.");
        
        // Attach the callback function
        DW3000.attachNewRange(newRangeCallback);
        
        // Start as Anchor (Example address "A1", using long data range accuracy mode)
        // Note: The ranging logic is complex and needs to be fully implemented in DW3000.cpp
        // This call currently only enables RX mode.
        DW3000.startAsAnchor("A1", MODE_LONGDATA_RANGE_ACCURACY);
        
    } else {
        Serial.println("DW3000 initialization failed. Check wiring and power.");
    }
}

void loop()
{
    // The main loop should handle the ranging state machine
    // For now, we just call the loop function of the DW3000 library
    DW3000.loop();
    
    // Add a small delay to prevent watchdog timer reset
    delay(10);
}
