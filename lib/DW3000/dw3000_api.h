/*
 * dw3000_api.h - DW3000 Minimal API Implementation
 * Based on Qorvo DW3000 Datasheet
 * Implements core SPI communication
 */

#ifndef DW3000_API_H
#define DW3000_API_H

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

/* ========== DW3000 SPI COMMAND HEADERS ========== */
#define DW3000_READ_CMD    0xC0   // Read bit
#define DW3000_WRITE_CMD   0x80   // Write bit
#define DW3000_WRITE_FAST  0xCF   // Fast write (special)

/* ========== DW3000 REGISTER ADDRESSES ========== */
#define DW_DEV_ID          0x00   // Device ID
#define DW_EUI             0x01   // Extended Unique ID
#define DW_PANADR          0x03   // PAN address
#define DW_SYS_CFG         0x04   // System config
#define DW_FF_CFG          0x05   // Frame filter
#define DW_SYS_TIME        0x06   // System time
#define DW_TX_FCTRL        0x08   // TX frame control
#define DW_TX_TIME         0x09   // TX timestamp
#define DW_RX_FWTO         0x0A   // RX frame wait timeout
#define DW_RX_CONFIG       0x0B   // RX config
#define DW_CH_CTRL         0x0D   // Channel control
#define DW_RX_FINFO        0x10   // RX frame info
#define DW_RX_FQUAL        0x12   // RX frame quality
#define DW_RX_TIME         0x15   // RX timestamp
#define DW_RX_DATA         0x11   // RX data buffer
#define DW_TX_DATA         0x14   // TX data buffer
#define DW_SYS_STATUS      0x03   // System status
#define DW_SYS_CTRL        0x0D   // System control
#define DW_DBLBUFFCON      0x0C   // Double buffer config
#define DW_OTP_IF          0x2D   // OTP interface
#define DW_GPIO_CTRL       0x26   // GPIO control
#define DW_LED_CTRL        0x28   // LED control
#define DW_RX_BUFFER       0x24   // RX buffer (extended address)

/* ========== STATUS REGISTER BITS ========== */
#define SYS_STATUS_RXFCG   0x00002000   // RX frame CRC good
#define SYS_STATUS_LDEDONE 0x00000001   // LDE processing done
#define SYS_STATUS_AAT     0x00001000   // Automatic ack trigger
#define SYS_STATUS_TXFRB   0x00000020   // TX frame sent
#define SYS_STATUS_RXPRD   0x00000100   // RX preamble detected
#define SYS_STATUS_RXSFDD  0x00000200   // RX SFD detected
#define SYS_STATUS_LCS     0x00000040   // LCS error

/* ========== SYSTEM CONTROL BITS ========== */
#define SYS_CTRL_RXE       0x00100000   // RX enable
#define SYS_CTRL_TRXE      0x00000200   // TX/RX enable
#define SYS_CTRL_SRXE      0x00000100   // Soft RX enable
#define SYS_CTRL_TXSTRT    0x00000002   // TX start
#define SYS_CTRL_RXRST     0x000E0000   // RX reset
#define SYS_CTRL_SFCST     0x00000004   // SFD clock stop
#define SYS_CTRL_CLKS      0x00000006   // Clock selection

class DW3000_SPI {
private:
    SPIClass *spi;
    uint8_t cs_pin;
    uint8_t irq_pin;
    uint8_t rst_pin;
    
    const SPISettings spi_settings = SPISettings(20000000, MSBFIRST, SPI_MODE0);
    
public:
    DW3000_SPI(uint8_t cs, uint8_t irq, uint8_t rst) 
        : cs_pin(cs), irq_pin(irq), rst_pin(rst), spi(&SPI) {}
    
    void init() {
        pinMode(cs_pin, OUTPUT);
        pinMode(irq_pin, INPUT);
        pinMode(rst_pin, OUTPUT);
        
        digitalWrite(cs_pin, HIGH);
        digitalWrite(rst_pin, HIGH);
        
        spi->begin();
    }
    
    void reset() {
        digitalWrite(rst_pin, LOW);
        delayMicroseconds(500);
        digitalWrite(rst_pin, HIGH);
        delay(300);
    }
    
    /* Read 4-byte register */
    uint32_t read32(uint16_t addr) {
        uint32_t result = 0;
        
        spi->beginTransaction(spi_settings);
        digitalWrite(cs_pin, LOW);
        delayMicroseconds(1);
        
        // Send address header (max 15-bit address)
        uint8_t header = DW3000_READ_CMD | ((addr >> 7) & 0x3F);
        uint8_t address = (addr & 0x7F) << 1;
        
        spi->transfer(header);
        spi->transfer(address);
        
        // Read 4 bytes
        for (int i = 0; i < 4; i++) {
            uint8_t byte = spi->transfer(0x00);
            result |= ((uint32_t)byte << (i * 8));
        }
        
        delayMicroseconds(1);
        digitalWrite(cs_pin, HIGH);
        spi->endTransaction();
        
        return result;
    }
    
    /* Write 4-byte register */
    void write32(uint16_t addr, uint32_t value) {
        spi->beginTransaction(spi_settings);
        digitalWrite(cs_pin, LOW);
        delayMicroseconds(1);
        
        // Send address header
        uint8_t header = DW3000_WRITE_CMD | ((addr >> 7) & 0x3F);
        uint8_t address = (addr & 0x7F) << 1;
        
        spi->transfer(header);
        spi->transfer(address);
        
        // Write 4 bytes
        for (int i = 0; i < 4; i++) {
            spi->transfer((value >> (i * 8)) & 0xFF);
        }
        
        delayMicroseconds(1);
        digitalWrite(cs_pin, HIGH);
        spi->endTransaction();
    }
    
    /* Read variable length data */
    void readData(uint16_t addr, uint8_t *buffer, uint16_t len) {
        spi->beginTransaction(spi_settings);
        digitalWrite(cs_pin, LOW);
        delayMicroseconds(1);
        
        uint8_t header = DW3000_READ_CMD | ((addr >> 7) & 0x3F);
        uint8_t address = (addr & 0x7F) << 1;
        
        spi->transfer(header);
        spi->transfer(address);
        
        for (uint16_t i = 0; i < len; i++) {
            buffer[i] = spi->transfer(0x00);
        }
        
        delayMicroseconds(1);
        digitalWrite(cs_pin, HIGH);
        spi->endTransaction();
    }
    
    /* Write variable length data */
    void writeData(uint16_t addr, const uint8_t *data, uint16_t len) {
        spi->beginTransaction(spi_settings);
        digitalWrite(cs_pin, LOW);
        delayMicroseconds(1);
        
        uint8_t header = DW3000_WRITE_CMD | ((addr >> 7) & 0x3F);
        uint8_t address = (addr & 0x7F) << 1;
        
        spi->transfer(header);
        spi->transfer(address);
        
        for (uint16_t i = 0; i < len; i++) {
            spi->transfer(data[i]);
        }
        
        delayMicroseconds(1);
        digitalWrite(cs_pin, HIGH);
        spi->endTransaction();
    }
    
    bool isIRQActive() {
        return digitalRead(irq_pin) == HIGH;
    }
};

#endif // DW3000_API_H
