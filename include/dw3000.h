/*
 * DW3000.h - Waveshare DW3000 Library
 * High-level wrapper for DW3000 chip with ranging support
 */

#ifndef DW3000_H
#define DW3000_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>
#include <string.h>

// DW3000 Operating modes
#define MODE_LONGDATA_RANGE_ACCURACY 0x01
#define MODE_SHORTDATA_FAST_ACCURACY 0x02
#define MODE_LONGDATA_FAST_ACCURACY  0x03

// DS-TWR Constants
#define POLL_RX_TO_RESP_TX_DLY_UUS 450
#define FINAL_RX_TIMEOUT_UUS 3000
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299792458.0
#define DWT_TIME_UNITS (1.0/499.2e6/128.0)

// RX/TX Antenna Delays
#define RX_ANT_DLY 16384
#define TX_ANT_DLY 16384

// Message Parameters
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 5

// Register Addresses
#define SYS_STATUS_ID 0x03
#define RX_FINFO_ID 0x07
#define RXFLEN_MASK 0x3FF

// Status Bits
#define SYS_STATUS_RXFCG_BIT_MASK 0x4000
#define SYS_STATUS_ALL_RX_ERR 0xFF00
#define SYS_STATUS_ALL_RX_TO 0x0800
#define SYS_STATUS_TXFRS_BIT_MASK 0x80

// Type definitions
typedef void (*DistanceCallback)();

class DW3000Class {
private:
    static const uint8_t SPI_BUFFER_LEN = 255;
    uint8_t SPI_buffer[SPI_BUFFER_LEN];
    
    uint8_t _rst_pin;
    uint8_t _ss_pin;
    uint8_t _irq_pin;
    
    uint64_t _device_address;
    float _last_distance;
    float _last_rssi;
    DistanceCallback _new_range_callback;
    
public:
    DW3000Class();
    
    // Initialization
    bool begin(uint8_t rst, uint8_t ss, uint8_t irq);
    bool startAsAnchor(const char* address, uint8_t mode);
    bool startAsTag(const char* address, uint8_t mode);
    
    // Main loop
    void loop();
    
    // SPI Communication
    void begin_spi();
    bool checkSPI();
    uint32_t read(uint16_t addr, uint16_t offset);
    void write(uint16_t addr, uint16_t offset, uint32_t data);
    void readBytes(uint16_t addr, uint16_t offset, uint8_t *buffer, uint16_t len);
    void writeBytes(uint16_t addr, uint16_t offset, const uint8_t *buffer, uint16_t len);
    
    // Chip Control
    void hardReset();
    void softReset();
    bool checkForIDLE();
    void init();
    
    // RX/TX Operations
    void standardRX();
    uint8_t receivedFrameSucc();
    void clearSystemStatus();
    void rxenable(uint8_t mode);
    void writetxdata(uint16_t len, const uint8_t* data, uint16_t offset);
    void writetxfctrl(uint16_t len, uint16_t offset, uint8_t flag);
    int starttx(uint8_t mode);
    uint64_t get_rx_timestamp_u64();
    void setdelayedtrxtime(uint32_t time);
    uint32_t read32bitreg(uint16_t addr);
    void write32bitreg(uint16_t addr, uint32_t data);
    uint32_t readrxdata(uint8_t *buffer, uint16_t len, uint16_t offset);
    
    // Range/Distance functions
    uint64_t getDistantDeviceAddress();
    float getDistantDeviceRange();
    float getDistantDeviceRSSI();
    
    // LED Control
    void setupGPIO();
    void pullLEDHigh(uint8_t pin);
    void pullLEDLow(uint8_t pin);
    void setleds(uint8_t mode);
    
    // Callbacks
    void attachNewRange(DistanceCallback callback);
};

extern DW3000Class DW3000;

#endif // DW3000_H

