/*
 * DW3000.cpp - Waveshare DW3000 Library Implementation
 * Implements SPI communication, chip control, and ranging
 */

#include "dw3000.h"

/* ========== PIN CONFIGURATION (from main.cpp) ========== */
// These are defined in main.cpp, but we use the pins passed to begin()

/* ========== DW3000 REGISTER ADDRESSES (from dw3000.h) ========== */
#define DEV_ID            0x00
#define SYS_CFG           0x04
#define SYS_STATUS        0x03
#define RX_FINFO          0x10
#define RX_DATA           0x24
#define TX_DATA           0x14
#define SYS_CTRL          0x0D
#define GPIO_CTRL         0x26
#define TX_FCTRL          0x08

/* ========== REGISTER BIT MASKS (from dw3000.h) ========== */
#define SYS_STATUS_RXFCG  0x00004000
#define SYS_CTRL_SFCST    0x00000004
#define SYS_CTRL_TRXOFF   0x00000040

DW3000Class DW3000;

DW3000Class::DW3000Class() : _new_range_callback(nullptr), _last_distance(0), _last_rssi(0) {
}

/* ================================================================= */
/* ======================== SPI COMMUNICATION ====================== */
/* ================================================================= */

// Helper function to create the SPI header byte
uint8_t create_header(uint16_t addr, uint16_t offset, bool is_read) {
    uint8_t header = 0;
    
    // Set R/W bit (Read=1, Write=0)
    if (is_read) {
        header |= 0x40;
    }
    
    // Set sub-addressing bit (Sub-addressing=1, No sub-addressing=0)
    if (offset != 0x00) {
        header |= 0x80;
    }
    
    // Set 7-bit address
    header |= (addr & 0x3F); // Only 6 bits for address in header
    
    return header;
}

void DW3000Class::readBytes(uint16_t addr, uint16_t offset, uint8_t *buffer, uint16_t len) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(_ss_pin, LOW);
    
    // Header byte: Read (bit 6), Sub-addressing (bit 7), Address (bits 0-5)
    uint8_t header = (1 << 6) | ((offset != 0x00) << 7) | (addr & 0x3F);
    SPI.transfer(header);
    
    // Sub-address bytes (if sub-addressing is enabled)
    if (offset != 0x00) {
        SPI.transfer((uint8_t)(offset & 0xFF));
        SPI.transfer((uint8_t)((offset >> 8) & 0xFF));
    }
    
    // Read data
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    
    digitalWrite(_ss_pin, HIGH);
    SPI.endTransaction();
}

void DW3000Class::writeBytes(uint16_t addr, uint16_t offset, const uint8_t *buffer, uint16_t len) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(_ss_pin, LOW);
    
    // Header byte: Write (bit 6=0), Sub-addressing (bit 7), Address (bits 0-5)
    uint8_t header = (0 << 6) | ((offset != 0x00) << 7) | (addr & 0x3F);
    SPI.transfer(header);
    
    // Sub-address bytes (if sub-addressing is enabled)
    if (offset != 0x00) {
        SPI.transfer((uint8_t)(offset & 0xFF));
        SPI.transfer((uint8_t)((offset >> 8) & 0xFF));
    }
    
    // Write data
    for (uint16_t i = 0; i < len; i++) {
        SPI.transfer(buffer[i]);
    }
    
    digitalWrite(_ss_pin, HIGH);
    SPI.endTransaction();
}

uint32_t DW3000Class::read(uint16_t addr, uint16_t offset) {
    uint8_t buffer[4];
    readBytes(addr, offset, buffer, 4);
    // DW3000 is Little Endian for 32-bit registers
    return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

void DW3000Class::write(uint16_t addr, uint16_t offset, uint32_t data) {
    uint8_t buffer[4];
    // DW3000 is Little Endian for 32-bit registers
    buffer[0] = (uint8_t)(data & 0xFF);
    buffer[1] = (uint8_t)((data >> 8) & 0xFF);
    buffer[2] = (uint8_t)((data >> 16) & 0xFF);
    buffer[3] = (uint8_t)((data >> 24) & 0xFF);
    writeBytes(addr, offset, buffer, 4);
}

// Helper functions for 32-bit register access (no offset)
uint32_t DW3000Class::read32bitreg(uint16_t addr) {
    return read(addr, 0x00);
}

void DW3000Class::write32bitreg(uint16_t addr, uint32_t data) {
    write(addr, 0x00, data);
}

/* ================================================================= */
/* ========================= CHIP CONTROL ========================== */
/* ================================================================= */

bool DW3000Class::begin(uint8_t rst, uint8_t ss, uint8_t irq) {
    _rst_pin = rst;
    _ss_pin = ss;
    _irq_pin = irq;
    
    pinMode(_ss_pin, OUTPUT);
    pinMode(_irq_pin, INPUT);
    pinMode(_rst_pin, OUTPUT);
    
    digitalWrite(_ss_pin, HIGH);
    digitalWrite(_rst_pin, HIGH);
    
    SPI.begin();
    Serial.println("[DW3000] SPI initialized");
    
    // Reset and check
    hardReset();
    delay(200);
    
    // Test SPI read
    uint32_t devID = read32bitreg(DEV_ID);
    Serial.printf("[DW3000] Device ID Read: 0x%08X\n", devID);
    
    // DW3000 Device ID is 0xDECA0301
    if (devID != 0xDECA0301) {
        Serial.println("[DW3000] Device ID check failed! Expected 0xDECA0301");
        return false;
    }
    
    Serial.println("[DW3000] SPI check passed!");
    
    // Initialize chip
    init();
    
    Serial.println("[DW3000] Initialization complete");
    return true;
}

void DW3000Class::hardReset() {
    digitalWrite(_rst_pin, LOW);
    delay(10);
    digitalWrite(_rst_pin, HIGH);
    delay(300);
    Serial.println("[DW3000] Hard reset complete");
}

void DW3000Class::softReset() {
    // Reset all registers
    write32bitreg(SYS_CTRL, SYS_CTRL_SFCST | SYS_CTRL_TRXOFF);
    delay(200);
    Serial.println("[DW3000] Soft reset complete");
}

void DW3000Class::init() {
    Serial.println("[DW3000] Initializing...");
    
    // Clear all status bits
    write32bitreg(SYS_STATUS, 0xFFFFFFFF);
    
    // Set system configuration (example: default settings)
    // This is a minimal configuration, full config is needed for ranging
    uint32_t sys_cfg = 0x00C08000; // Example value, needs proper config
    write32bitreg(SYS_CFG, sys_cfg);
    
    // Set antenna delays
    write(0x02, 0x04, TX_ANT_DLY); // TX_ANT_DLY (register 0x02, offset 0x04)
    write(0x02, 0x06, RX_ANT_DLY); // RX_ANT_DLY (register 0x02, offset 0x06)
    
    delay(100);
    Serial.println("[DW3000] Initialization complete");
}

/* ================================================================= */
/* ======================= RX/TX OPERATIONS ======================== */
/* ================================================================= */

void DW3000Class::standardRX() {
    // Clear all status bits
    write32bitreg(SYS_STATUS, 0xFFFFFFFF);
    
    // Enable RX (RXENAB bit in SYS_CTRL)
    uint32_t ctrl = read32bitreg(SYS_CTRL);
    ctrl |= 0x00100000; // RXENAB bit
    write32bitreg(SYS_CTRL, ctrl);
}

uint8_t DW3000Class::receivedFrameSucc() {
    uint32_t status = read32bitreg(SYS_STATUS);
    
    if ((status & SYS_STATUS_RXFCG) != 0) {
        return 1; // Frame received successfully
    } else if ((status & 0x00080000) != 0) {
        return 2; // RX Timeout (example bit, needs verification)
    }
    return 0;
}

void DW3000Class::clearSystemStatus() {
    write32bitreg(SYS_STATUS, 0xFFFFFFFF);
}

void DW3000Class::rxenable(uint8_t mode) {
    // Enable RX mode (same as standardRX for now, mode is ignored)
    standardRX();
}

void DW3000Class::writetxdata(uint16_t len, const uint8_t* data, uint16_t offset) {
    writeBytes(TX_DATA, offset, data, len);
}

void DW3000Class::writetxfctrl(uint16_t len, uint16_t offset, uint8_t flag) {
    uint32_t ctrl = (len << 16) | offset;
    if (flag) ctrl |= 0x80000000; // Enable TX flag (needs verification)
    write32bitreg(TX_FCTRL, ctrl);
}

int DW3000Class::starttx(uint8_t mode) {
    uint32_t ctrl = read32bitreg(SYS_CTRL);
    ctrl |= 0x00000002; // TXSTRT bit
    write32bitreg(SYS_CTRL, ctrl);
    return 0;
}

uint64_t DW3000Class::get_rx_timestamp_u64() {
    uint8_t ts_buffer[5];
    // RX_TIME register is 0x15
    readBytes(RX_TIME, 0x00, ts_buffer, 5);
    
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++) {
        ts |= ((uint64_t)ts_buffer[i]) << (i * 8);
    }
    return ts;
}

void DW3000Class::setdelayedtrxtime(uint32_t time) {
    // DX_TIME register is 0x11
    write32bitreg(0x11, time);
}

uint32_t DW3000Class::readrxdata(uint8_t *buffer, uint16_t len, uint16_t offset) {
    readBytes(RX_DATA, offset, buffer, len);
    return len;
}

/* ================================================================= */
/* ========================= RANGING LOGIC ========================= */
/* ================================================================= */

bool DW3000Class::startAsAnchor(const char* address, uint8_t mode) {
    Serial.printf("[DW3000] Starting as Anchor with address: %s\n", address);
    
    // Anchor specific setup (needs full ranging logic)
    // For now, just initialize and enable RX
    standardRX();
    
    return true;
}

bool DW3000Class::startAsTag(const char* address, uint8_t mode) {
    Serial.printf("[DW3000] Starting as Tag with Anchor address: %s\n", address);
    
    // Tag specific setup (needs full ranging logic)
    // For now, just initialize
    
    return true;
}

void DW3000Class::loop() {
    // Placeholder for ranging logic (e.g., checking for interrupts, processing frames)
    // The actual ranging logic is complex and requires state machine implementation.
    
    // Example: Check for received frames
    if (receivedFrameSucc() == 1) {
        // Frame received successfully
        // ... process frame ...
        
        // Clear status and re-enable RX
        clearSystemStatus();
        standardRX();
        
        // Placeholder for distance calculation
        _last_distance = 1.0f; // Example value
        _last_rssi = -70.0f; // Example value
        
        // Call callback if set
        if (_new_range_callback != nullptr) {
            _new_range_callback();
        }
    }
}

/* ================================================================= */
/* ========================= GETTERS/SETTERS ======================= */
/* ================================================================= */

uint64_t DW3000Class::getDistantDeviceAddress() {
    return _device_address;
}

float DW3000Class::getDistantDeviceRange() {
    return _last_distance;
}

float DW3000Class::getDistantDeviceRSSI() {
    return _last_rssi;
}

void DW3000Class::setupGPIO() {
    // Placeholder for GPIO setup
    Serial.println("[DW3000] GPIO setup complete (Placeholder)");
}

void DW3000Class::setleds(uint8_t mode) {
    // Placeholder for LED control
}

void DW3000Class::attachNewRange(DistanceCallback callback) {
    _new_range_callback = callback;
    Serial.println("[DW3000] New range callback attached");
}
