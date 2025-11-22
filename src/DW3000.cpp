/*
 * DW3000.cpp - Waveshare DW3000 Library Implementation
 * Implements SPI communication, chip control, and ranging
 */

#include "DW3000.h"

/* ========== PIN CONFIGURATION ========== */
#define DW3000_CS    4     // Chip Select (GPIO4)
#define DW3000_IRQ   34    // Interrupt (GPIO34)
#define DW3000_RST   27    // Reset (GPIO27)

/* ========== DW3000 REGISTER ADDRESSES ========== */
#define DEV_ID            0x00
#define SYS_CFG           0x04
#define SYS_STATUS        0x03
#define RX_FINFO          0x10
#define RX_DATA           0x24
#define TX_DATA           0x14
#define SYS_CTRL          0x0D
#define GPIO_CTRL         0x26
#define TX_FCTRL          0x08

/* ========== REGISTER BIT MASKS ========== */
#define SYS_STATUS_RXFCG  0x00002000
#define SYS_STATUS_RXPRD  0x00000100
#define SYS_STATUS_RXSFDD 0x00000200
#define SYS_STATUS_RXFR   0x00004000
#define SYS_CTRL_SFCST    0x00000004
#define SYS_CTRL_TRXOFF   0x00000040

/* ========== SPI SETTINGS ========== */
SPISettings spiSettings(20000000, MSBFIRST, SPI_MODE0);

DW3000Class DW3000;

DW3000Class::DW3000Class() : _new_range_callback(nullptr), _last_distance(0), _last_rssi(0) {
}

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
    uint32_t devID = read(DEV_ID, 0x00);
    Serial.printf("[DW3000] Device ID Read: 0x%08X\n", devID);
    
    if ((devID & 0xFFFF0000) != 0xDECA0000) {
        Serial.println("[DW3000] Device ID check failed!");
        return false;
    }
    
    Serial.println("[DW3000] SPI check passed!");
    
    // Initialize chip
    init();
    
    Serial.println("[DW3000] Initialization complete");
    return true;
}

bool DW3000Class::startAsAnchor(const char* address, uint8_t mode) {
    Serial.printf("[DW3000] Starting as Anchor with address: %s\n", address);
    
    // Parse address (simple hex parsing)
    // In real implementation, this would parse the MAC address string
    
    setupGPIO();
    clearSystemStatus();
    
    return true;
}

bool DW3000Class::startAsTag(const char* address, uint8_t mode) {
    Serial.printf("[DW3000] Starting as Tag with Anchor address: %s\n", address);
    
    setupGPIO();
    clearSystemStatus();
    
    return true;
}

void DW3000Class::loop() {
    // Check for received frames
    if (receivedFrameSucc() == 1) {
        // Frame received successfully
        uint32_t frame_data = read(0x12, 0x00);
        
        // Calculate distance (simplified)
        _last_distance = (frame_data & 0xFFFF) / 100.0f;
        _last_rssi = ((frame_data >> 16) & 0xFF) - 120.0f;
        
        clearSystemStatus();
        
        // Call callback if set
        if (_new_range_callback != nullptr) {
            _new_range_callback();
        }
    }
    
    // Re-enable RX
    standardRX();
}

void DW3000Class::begin_spi() {
    pinMode(_ss_pin, OUTPUT);
    pinMode(_irq_pin, INPUT);
    pinMode(_rst_pin, OUTPUT);
    
    digitalWrite(_ss_pin, HIGH);
    digitalWrite(_rst_pin, HIGH);
    
    SPI.begin();
}

bool DW3000Class::checkSPI() {
    uint32_t devID = read(DEV_ID, 0x00);
    Serial.printf("[DW3000] Device ID: 0x%08X\n", devID);
    
    if ((devID & 0xFFFF0000) == 0xDECA0000) {
        Serial.println("[DW3000] SPI Check OK");
        return true;
    }
    return false;
}

uint32_t DW3000Class::read(uint16_t addr, uint16_t offset) {
    uint32_t result = 0;
    
    SPI.beginTransaction(spiSettings);
    digitalWrite(_ss_pin, LOW);
    delayMicroseconds(1);
    
    uint8_t header = 0xC0 | ((addr >> 7) & 0x3F);
    uint8_t address = ((addr & 0x7F) << 1) | ((offset >> 8) & 0x01);
    
    SPI.transfer(header);
    SPI.transfer(address);
    SPI.transfer(offset & 0xFF);
    
    for (int i = 0; i < 4; i++) {
        uint8_t byte = SPI.transfer(0x00);
        result |= ((uint32_t)byte << (i * 8));
    }
    
    delayMicroseconds(1);
    digitalWrite(_ss_pin, HIGH);
    SPI.endTransaction();
    
    return result;
}

void DW3000Class::write(uint16_t addr, uint16_t offset, uint32_t data) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(_ss_pin, LOW);
    delayMicroseconds(1);
    
    uint8_t header = 0x80 | ((addr >> 7) & 0x3F);
    uint8_t address = ((addr & 0x7F) << 1) | ((offset >> 8) & 0x01);
    
    SPI.transfer(header);
    SPI.transfer(address);
    SPI.transfer(offset & 0xFF);
    
    for (int i = 0; i < 4; i++) {
        SPI.transfer((data >> (i * 8)) & 0xFF);
    }
    
    delayMicroseconds(1);
    digitalWrite(_ss_pin, HIGH);
    SPI.endTransaction();
}

void DW3000Class::readBytes(uint16_t addr, uint16_t offset, uint8_t *buffer, uint16_t len) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(_ss_pin, LOW);
    delayMicroseconds(1);
    
    uint8_t header = 0xC0 | ((addr >> 7) & 0x3F);
    uint8_t address = ((addr & 0x7F) << 1) | ((offset >> 8) & 0x01);
    
    SPI.transfer(header);
    SPI.transfer(address);
    SPI.transfer(offset & 0xFF);
    
    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    
    delayMicroseconds(1);
    digitalWrite(_ss_pin, HIGH);
    SPI.endTransaction();
}

void DW3000Class::writeBytes(uint16_t addr, uint16_t offset, const uint8_t *buffer, uint16_t len) {
    SPI.beginTransaction(spiSettings);
    digitalWrite(_ss_pin, LOW);
    delayMicroseconds(1);
    
    uint8_t header = 0x80 | ((addr >> 7) & 0x3F);
    uint8_t address = ((addr & 0x7F) << 1) | ((offset >> 8) & 0x01);
    
    SPI.transfer(header);
    SPI.transfer(address);
    SPI.transfer(offset & 0xFF);
    
    for (uint16_t i = 0; i < len; i++) {
        SPI.transfer(buffer[i]);
    }
    
    delayMicroseconds(1);
    digitalWrite(_ss_pin, HIGH);
    SPI.endTransaction();
}

void DW3000Class::hardReset() {
    digitalWrite(_rst_pin, LOW);
    delay(10);
    digitalWrite(_rst_pin, HIGH);
    delay(300);
    Serial.println("[DW3000] Hard reset complete");
}

void DW3000Class::softReset() {
    uint32_t ctrl = read(SYS_CTRL, 0x00);
    ctrl |= SYS_CTRL_SFCST | SYS_CTRL_TRXOFF;
    write(SYS_CTRL, 0x00, ctrl);
    delay(200);
    Serial.println("[DW3000] Soft reset complete");
}

bool DW3000Class::checkForIDLE() {
    uint32_t status = read(SYS_STATUS, 0x00);
    bool idle = (status & 0x00000004) != 0;
    
    if (!idle) {
        Serial.printf("[DW3000] Not IDLE - Status: 0x%08X\n", status);
    }
    return idle;
}

void DW3000Class::init() {
    Serial.println("[DW3000] Initializing...");
    
    uint32_t sys_cfg = 0x00C08000;
    write(SYS_CFG, 0x00, sys_cfg);
    
    write(SYS_STATUS, 0x00, 0xFFFFFFFF);
    
    delay(100);
    Serial.println("[DW3000] Initialization complete");
}

void DW3000Class::standardRX() {
    write(SYS_STATUS, 0x00, 0xFFFFFFFF);
    
    uint32_t ctrl = read(SYS_CTRL, 0x00);
    ctrl |= 0x00100000;
    write(SYS_CTRL, 0x00, ctrl);
}

uint8_t DW3000Class::receivedFrameSucc() {
    uint32_t status = read(SYS_STATUS, 0x00);
    
    if ((status & SYS_STATUS_RXFCG) != 0) {
        return 1;
    } else if ((status & 0x00080000) != 0) {
        return 2;
    }
    return 0;
}

void DW3000Class::clearSystemStatus() {
    write(SYS_STATUS, 0x00, 0xFFFFFFFF);
}

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
    uint32_t gpio_ctrl = 0x00000000;
    write(GPIO_CTRL, 0x00, gpio_ctrl);
    Serial.println("[DW3000] GPIO setup complete");
}

void DW3000Class::pullLEDHigh(uint8_t pin) {
    uint32_t gpio = read(GPIO_CTRL, 0x00);
    gpio |= (1 << (pin * 2));
    write(GPIO_CTRL, 0x00, gpio);
}

void DW3000Class::pullLEDLow(uint8_t pin) {
    uint32_t gpio = read(GPIO_CTRL, 0x00);
    gpio &= ~(1 << (pin * 2));
    write(GPIO_CTRL, 0x00, gpio);
}

void DW3000Class::attachNewRange(DistanceCallback callback) {
    _new_range_callback = callback;
    Serial.println("[DW3000] New range callback attached");
}

// ========== DS-TWR Helper Functions ==========

void DW3000Class::rxenable(uint8_t mode) {
    // Enable RX mode
    uint32_t ctrl = read32bitreg(SYS_CTRL);
    ctrl |= 0x00000100; // RXENAB bit
    write32bitreg(SYS_CTRL, ctrl);
}

void DW3000Class::writetxdata(uint16_t len, const uint8_t* data, uint16_t offset) {
    writeBytes(TX_DATA, offset, data, len);
}

void DW3000Class::writetxfctrl(uint16_t len, uint16_t offset, uint8_t flag) {
    uint32_t ctrl = (len << 16) | offset;
    if (flag) ctrl |= 0x80000000; // Enable TX flag
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
    readBytes(0x15, 0x00, ts_buffer, 5);
    
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++) {
        ts |= ((uint64_t)ts_buffer[i]) << (i * 8);
    }
    return ts;
}

void DW3000Class::setdelayedtrxtime(uint32_t time) {
    write32bitreg(0x11, time);
}

uint32_t DW3000Class::read32bitreg(uint16_t addr) {
    return read(addr, 0x00);
}

void DW3000Class::write32bitreg(uint16_t addr, uint32_t data) {
    write(addr, 0x00, data);
}

uint32_t DW3000Class::readrxdata(uint8_t *buffer, uint16_t len, uint16_t offset) {
    readBytes(RX_DATA, offset, buffer, len);
    return len;
}

void DW3000Class::setleds(uint8_t mode) {
    // LED control - simplified for now
    if (mode & 0x01) {
        pullLEDHigh(1);
    }
}
