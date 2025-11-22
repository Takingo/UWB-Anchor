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

    // Direct SPI test
    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_IRQ, INPUT);
    pinMode(PIN_RST, OUTPUT);
    
    digitalWrite(PIN_SS, HIGH);
    digitalWrite(PIN_RST, HIGH);
    delay(100);
    
    // Hard reset
    digitalWrite(PIN_RST, LOW);
    delay(10);
    digitalWrite(PIN_RST, HIGH);
    delay(300);
    
    Serial.println("Hard reset done, testing SPI read...");
    
    // Test SPI read
    SPI.begin();
    SPISettings settings(20000000, MSBFIRST, SPI_MODE0);
    
    SPI.beginTransaction(settings);
    digitalWrite(PIN_SS, LOW);
    delayMicroseconds(1);
    
    // Read Device ID (address 0x00)
    uint8_t header = 0xC0 | 0x00;  // Read, address 0
    uint8_t address = 0x00;
    uint8_t offset = 0x00;
    
    SPI.transfer(header);
    SPI.transfer(address);
    SPI.transfer(offset);
    
    uint32_t devID = 0;
    for (int i = 0; i < 4; i++) {
        uint8_t byte = SPI.transfer(0x00);
        devID |= ((uint32_t)byte << (i * 8));
        Serial.printf("Byte %d: 0x%02X\n", i, byte);
    }
    
    delayMicroseconds(1);
    digitalWrite(PIN_SS, HIGH);
    SPI.endTransaction();
    
    Serial.printf("Device ID: 0x%08X\n", devID);
    
    if ((devID & 0xFFFF0000) == 0xDECA0000) {
        Serial.println("SUCCESS: Device ID is correct!");
    } else {
        Serial.println("ERROR: Device ID is incorrect!");
    }
}

void loop()
{
    delay(1000);
}