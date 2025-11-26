/**
 * ============================================================================
 * ESP32-S3 + DWM3000 UWB Anchor - Phase 1
 * ============================================================================
 * 
 * Bu firmware, ESP32-S3-DevKitC-1 kartını DWM3000 UWB modülü ile kullanarak
 * K4W UWB Tag'dan gelen Blink frame'lerini dinler.
 * 
 * DONANIM BAĞLANTILARI:
 * ---------------------
 * DWM3000 Pin    | ESP32-S3 GPIO | Açıklama
 * ---------------|---------------|----------------------------------
 * VDD (3.3V)     | 3.3V          | Güç kaynağı
 * GND            | GND           | Toprak
 * SPICLK         | GPIO12        | SPI Clock
 * SPIMOSI        | GPIO11        | SPI Master Out Slave In
 * SPIMISO        | GPIO13        | SPI Master In Slave Out
 * SPICSn         | GPIO10        | SPI Chip Select (Active LOW)
 * IRQ            | GPIO4         | Interrupt Request
 * RSTn           | GPIO5         | Reset (Active LOW)
 * EXTON          | 3.3V          | Power Enable (bağlı tut)
 * 
 * ÖZELLİKLER:
 * -----------
 * - DWM3000 SPI iletişimi
 * - Device ID okuma ve doğrulama
 * - K4W Tag Blink frame dinleme
 * - Frame alındığında RGB LED yanıp sönme
 * - Serial Monitor debug bilgileri
 * - Gerçek zamanlı frame sayacı
 * 
 * LED FEEDBACK:
 * -------------
 * - RGB LED (GPIO 38): Frame alındığında YEŞIL yanar
 * - Yanma süresi: 100ms
 * - Her frame için yeni yanma
 * 
 * SERIAL MONITOR:
 * ---------------
 * - Baud rate: 115200
 * - Device ID bilgisi
 * - Frame alındığında detaylı bilgi
 * - Frame sayısı ve hata sayısı
 * - Her 10 saniyede istatistik
 * 
 * YAZILIM:
 * --------
 * - Framework: Arduino (ESP-IDF tabanlı)
 * - Platform: PlatformIO
 * - Kütüphaneler: SPI, Adafruit_NeoPixel
 * 
 * GELIŞTIRME:
 * -----------
 * - Profesör: UWB RTLS Projesi
 * - Manus AI: Firmware Geliştirme
 * - Tarih: 26 Kasım 2025
 * - Versiyon: 1.0.0 (Phase 1)
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include "../include/dwm3000_regs.h"

// ============================================================================
// PIN TANIMLARI
// ============================================================================

// DWM3000 SPI Pinleri
#define DWM3000_CS_PIN      10
#define DWM3000_SCLK_PIN    12
#define DWM3000_MOSI_PIN    11
#define DWM3000_MISO_PIN    13
#define DWM3000_IRQ_PIN     4
#define DWM3000_RST_PIN     5

// RGB LED (WS2812)
#define RGB_LED_PIN         38
#define NUM_PIXELS          1

// ============================================================================
// GLOBAL NESNELER
// ============================================================================

SPIClass *spi = nullptr;
Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// GLOBAL DEĞİŞKENLER
// ============================================================================

volatile bool frame_received = false;  // IRQ flag
volatile uint32_t irq_count = 0;       // IRQ tetiklenme sayısı (debug için)
uint32_t frame_count = 0;              // Toplam alınan frame sayısı
uint32_t error_count = 0;              // CRC hata sayısı
uint32_t last_frame_time = 0;          // Son frame zamanı
bool dwm3000_initialized = false;      // DWM3000 başlatıldı mı?

// ============================================================================
// DWM3000 SPI FONKSİYONLARI
// ============================================================================

/**
 * DWM3000'e register yaz
 */
void dwm3000_write_reg(uint16_t reg_addr, uint8_t *data, uint16_t len) {
    digitalWrite(DWM3000_CS_PIN, LOW);
    delayMicroseconds(1);
    
    // Write command (bit 7 = 1)
    spi->transfer(DWM3000_WRITE_REG(reg_addr));
    
    // Data bytes
    for (uint16_t i = 0; i < len; i++) {
        spi->transfer(data[i]);
    }
    
    delayMicroseconds(1);
    digitalWrite(DWM3000_CS_PIN, HIGH);
}

/**
 * DWM3000'den register oku
 */
void dwm3000_read_reg(uint16_t reg_addr, uint8_t *data, uint16_t len) {
    digitalWrite(DWM3000_CS_PIN, LOW);
    delayMicroseconds(1);
    
    // Read command (bit 7 = 0)
    spi->transfer(DWM3000_READ_REG(reg_addr));
    
    // Data bytes
    for (uint16_t i = 0; i < len; i++) {
        data[i] = spi->transfer(0x00);
    }
    
    delayMicroseconds(1);
    digitalWrite(DWM3000_CS_PIN, HIGH);
}

/**
 * DWM3000 Device ID oku
 */
uint32_t dwm3000_read_device_id() {
    uint8_t buffer[4];
    dwm3000_read_reg(DWM3000_REG_DEV_ID, buffer, 4);
    
    return (uint32_t)buffer[0] | 
           ((uint32_t)buffer[1] << 8) | 
           ((uint32_t)buffer[2] << 16) | 
           ((uint32_t)buffer[3] << 24);
}

/**
 * DWM3000'i hardware reset et
 */
void dwm3000_reset() {
    digitalWrite(DWM3000_RST_PIN, LOW);
    delay(DWM3000_RESET_DELAY_MS);
    digitalWrite(DWM3000_RST_PIN, HIGH);
    delay(DWM3000_INIT_DELAY_MS);
}

/**
 * DWM3000 System Status oku
 */
uint32_t dwm3000_read_sys_status() {
    uint8_t buffer[4];
    dwm3000_read_reg(DWM3000_REG_SYS_STATUS, buffer, 4);
    
    return (uint32_t)buffer[0] | 
           ((uint32_t)buffer[1] << 8) | 
           ((uint32_t)buffer[2] << 16) | 
           ((uint32_t)buffer[3] << 24);
}

/**
 * System Status'u temizle
 */
void dwm3000_clear_sys_status(uint32_t mask) {
    uint8_t cmd[4] = {
        (uint8_t)(mask & 0xFF),
        (uint8_t)((mask >> 8) & 0xFF),
        (uint8_t)((mask >> 16) & 0xFF),
        (uint8_t)((mask >> 24) & 0xFF)
    };
    dwm3000_write_reg(DWM3000_REG_SYS_STATUS, cmd, 4);
}

/**
 * RX Buffer'dan frame oku
 */
void dwm3000_read_rx_buffer(uint8_t *buffer, uint16_t len) {
    dwm3000_read_reg(DWM3000_REG_RX_BUFFER, buffer, len);
}

/**
 * DWM3000'i RX moduna al
 */
void dwm3000_enable_rx() {
    uint8_t cmd[4] = {
        (uint8_t)(SYS_CTRL_RXENAB & 0xFF),
        (uint8_t)((SYS_CTRL_RXENAB >> 8) & 0xFF),
        (uint8_t)((SYS_CTRL_RXENAB >> 16) & 0xFF),
        (uint8_t)((SYS_CTRL_RXENAB >> 24) & 0xFF)
    };
    dwm3000_write_reg(DWM3000_REG_SYS_CTRL, cmd, 4);
}

/**
 * DWM3000'i konfigüre et (K4W Tag TAM UYUMLU)
 * K4W Tag: Ch5, PRF64, 850kbps (default), Preamble 128, Code 9
 */
void dwm3000_configure() {
    Serial.println("[CONFIG] K4W Tag compatible configuration...");
    
    // 1. System Config: RXAUTR + DIS_DRXB (continuous RX)
    uint8_t sys_cfg[4] = {
        0x00,  
        0x00,
        0x00,  
        0x20   // RXAUTR=1 (Auto RX re-enable)
    };
    dwm3000_write_reg(DWM3000_REG_SYS_CFG, sys_cfg, 4);
    Serial.println("[CONFIG] SYS_CFG: Auto RX");
    
    // 2. Channel 5, PRF 64 MHz, Preamble Code 9
    uint8_t chan_ctrl[4] = {
        0x55,  // TX Ch5, RX Ch5
        0xC9,  // PRF 64MHz (11xxxxxx) + Preamble Code 9 (xx001001)
        0x00,  
        0x00
    };
    dwm3000_write_reg(DWM3000_REG_CHAN_CTRL, chan_ctrl, 4);
    Serial.println("[CONFIG] CHAN_CTRL: Ch5, PRF64, Code9");
    
    // 3. TX Frame Control: 850kbps, Preamble 128
    uint8_t tx_fctrl[5] = {
        0x00,  
        0x00,
        0x08,  // TXPSR=01 (for 850k), PE=00 (128 symbols) 
        0x10,  // TXBR=01 (850 kbps - NOT 6.8M!)
        0x00
    };
    dwm3000_write_reg(DWM3000_REG_TX_FCTRL, tx_fctrl, 5);
    Serial.println("[CONFIG] TX_FCTRL: Preamble 128, 850kbps");
    
    // 4. DRX Config: PAC 8 for preamble 128
    uint8_t drx_conf[2] = {0x02, 0x00};  
    dwm3000_write_reg(0x27, drx_conf, 2);
    Serial.println("[CONFIG] DRX: PAC 8");
    
    // 5. RX Frame Wait Timeout
    uint8_t rx_fwto[2] = {0x00, 0x20};
    dwm3000_write_reg(0x0C, rx_fwto, 2);
    
    // 6. IRQ Mask: ALL RX events
    uint8_t sys_mask[4] = {
        (uint8_t)((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXPRD) & 0xFF),
        (uint8_t)(((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXPRD) >> 8) & 0xFF),
        (uint8_t)(((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXPRD) >> 16) & 0xFF),
        (uint8_t)(((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXPRD) >> 24) & 0xFF)
    };
    dwm3000_write_reg(DWM3000_REG_SYS_MASK, sys_mask, 4);
    Serial.println("[CONFIG] IRQ Mask: RX events");
    
    // 7. Clear all status bits
    uint8_t clear_all[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    dwm3000_write_reg(DWM3000_REG_SYS_STATUS, clear_all, 4);
    
    Serial.println("[OK] Ch5, PRF64, 850k, P128, Code9");
}

// ============================================================================
// IRQ HANDLER
// ============================================================================

void IRAM_ATTR dwm3000_irq_handler() {
    irq_count++;  // Debug: IRQ sayısını artır
    frame_received = true;
}

// ============================================================================
// LED FONKSİYONLARI
// ============================================================================

/**
 * RGB LED'i yeşil yak (frame alındı)
 */
void led_frame_received() {
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));  // Yeşil
    pixels.show();
    delay(100);
    pixels.clear();
    pixels.show();
}

/**
 * RGB LED'i kırmızı yak (hata)
 */
void led_error() {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));  // Kırmızı
    pixels.show();
    delay(100);
    pixels.clear();
    pixels.show();
}

/**
 * RGB LED'i mavi yak (başlatma)
 */
void led_init() {
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));  // Mavi
    pixels.show();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Serial başlat
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n========================================");
    Serial.println("ESP32-S3 + DWM3000 UWB Anchor - Phase 1");
    Serial.println("========================================");
    Serial.printf("CPU: %d MHz | RAM: %d KB | Flash: %d MB\n", 
                  ESP.getCpuFreqMHz(), 
                  ESP.getFreeHeap() / 1024,
                  ESP.getFlashChipSize() / (1024 * 1024));
    Serial.println("========================================\n");
    
    // RGB LED başlat
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();
    led_init();
    
    // Pin ayarları
    pinMode(DWM3000_CS_PIN, OUTPUT);
    pinMode(DWM3000_RST_PIN, OUTPUT);
    pinMode(DWM3000_IRQ_PIN, INPUT);
    
    digitalWrite(DWM3000_CS_PIN, HIGH);
    digitalWrite(DWM3000_RST_PIN, HIGH);
    
    // SPI başlat
    Serial.println("[INIT] Initializing SPI...");
    spi = new SPIClass(FSPI);
    spi->begin(DWM3000_SCLK_PIN, DWM3000_MISO_PIN, DWM3000_MOSI_PIN, DWM3000_CS_PIN);
    spi->setFrequency(DWM3000_SPI_SPEED_SLOW);  // 2 MHz (başlangıç)
    spi->setDataMode(SPI_MODE0);
    spi->setBitOrder(MSBFIRST);
    Serial.println("[OK] SPI initialized at 2 MHz");
    
    // DWM3000 reset
    Serial.println("[RESET] Resetting DWM3000...");
    dwm3000_reset();
    Serial.println("[OK] Reset complete");
    
    // Device ID oku
    Serial.println("[INIT] Reading Device ID...");
    uint32_t dev_id = dwm3000_read_device_id();
    Serial.printf("[INFO] Device ID: 0x%08X\n", dev_id);
    
    // Device ID kontrolü
    if (dev_id == 0x00000000 || dev_id == 0xFFFFFFFF) {
        Serial.println("[ERROR] Invalid Device ID! Check SPI connections.");
        Serial.println("Expected: 0xDECA0302 or 0xDECA0301");
        Serial.println("\nTroubleshooting:");
        Serial.println("1. Check SPI wiring (SCLK, MOSI, MISO, CS)");
        Serial.println("2. Verify 3.3V power to DWM3000");
        Serial.println("3. Check EXTON pin (must be HIGH or 3.3V)");
        Serial.println("4. Measure SPI signals with logic analyzer");
        
        while (1) {
            led_error();
            delay(500);
        }
    }
    
    Serial.println("[OK] DWM3000 detected successfully!");
    
    // DWM3000 konfigüre et
    Serial.println("[CONFIG] Configuring DWM3000...");
    dwm3000_configure();
    Serial.println("[OK] Configuration complete");
    
    // IRQ interrupt mask ayarla (SYS_ENABLE register)
    Serial.println("[CONFIG] Enabling IRQ interrupts...");
    uint8_t sys_enable[4] = {
        (uint8_t)((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE) & 0xFF),
        (uint8_t)(((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE) >> 8) & 0xFF),
        (uint8_t)(((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE) >> 16) & 0xFF),
        (uint8_t)(((SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE) >> 24) & 0xFF)
    };
    dwm3000_write_reg(DWM3000_REG_SYS_ENABLE, sys_enable, 4);
    Serial.println("[OK] IRQ interrupts enabled (RXFCG, RXFCE, RXPHE)");
    
    // IRQ interrupt ayarla (ESP32 GPIO)
    pinMode(DWM3000_IRQ_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(DWM3000_IRQ_PIN), dwm3000_irq_handler, RISING);
    Serial.printf("[OK] IRQ interrupt configured on GPIO %d\n", DWM3000_IRQ_PIN);
    
    // RX modunu etkinleştir
    Serial.println("[RX] Enabling receiver...");
    dwm3000_enable_rx();
    Serial.println("[OK] Receiver enabled");
    
    dwm3000_initialized = true;
    
    Serial.println("\n========================================");
    Serial.println("[READY] Listening for K4W Tag frames...");
    Serial.println("========================================\n");
    
    // Başlangıç LED animasyonu (3x yeşil blink)
    pixels.clear();
    pixels.show();
    for (int i = 0; i < 3; i++) {
        led_frame_received();
        delay(200);
    }
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    // Frame alındı mı kontrol et
    if (frame_received && dwm3000_initialized) {
        frame_received = false;
        
        // System Status oku
        uint32_t sys_status = dwm3000_read_sys_status();
        
        Serial.printf("[IRQ] Status: 0x%08X\n", sys_status);
        
        // Frame başarıyla alındı mı?
        if (sys_status & SYS_STATUS_RXFCG) {
            frame_count++;
            last_frame_time = millis();
            
            // LED feedback
            led_frame_received();
            
            // RX Buffer'dan frame oku
            uint8_t rx_buffer[127];
            dwm3000_read_rx_buffer(rx_buffer, 20);  // İlk 20 byte
            
            // Frame bilgilerini yazdır
            Serial.printf("[RX] Frame #%u received at %lu ms\n", frame_count, last_frame_time);
            Serial.print("     Data: ");
            for (int i = 0; i < 15; i++) {
                Serial.printf("%02X ", rx_buffer[i]);
            }
            Serial.println();
            
            // Status'u temizle
            dwm3000_clear_sys_status(SYS_STATUS_RXFCG);
            
            // RX'i yeniden etkinleştir
            dwm3000_enable_rx();
            
        } else if (sys_status & SYS_STATUS_RXFCE) {
            // CRC hatası
            error_count++;
            Serial.printf("[ERROR] Frame CRC error (Total errors: %u)\n", error_count);
            
            // Status'u temizle
            dwm3000_clear_sys_status(SYS_STATUS_RXFCE);
            
            // RX'i yeniden etkinleştir
            dwm3000_enable_rx();
            
        } else if (sys_status & SYS_STATUS_RXPHE) {
            // PHY hatası
            static uint32_t phy_error_count = 0;
            phy_error_count++;
            if (phy_error_count % 10 == 1) {  // Her 10 PHY hatasında bir yazdır
                Serial.printf("[WARN] PHY error detected (count: %u) - possible interference or wrong preamble\n", phy_error_count);
            }
            
            // PHY hatasını temizle
            uint8_t clear_phy[4] = {
                (uint8_t)(SYS_STATUS_RXPHE & 0xFF),
                (uint8_t)((SYS_STATUS_RXPHE >> 8) & 0xFF),
                (uint8_t)((SYS_STATUS_RXPHE >> 16) & 0xFF),
                (uint8_t)((SYS_STATUS_RXPHE >> 24) & 0xFF)
            };
            dwm3000_write_reg(DWM3000_REG_SYS_STATUS, clear_phy, 4);
            
            // RX'i yeniden etkinleştir
            dwm3000_enable_rx();
        }
    }
    
    // Her 5 saniyede bir System Status debug
    static uint32_t last_status_check = 0;
    if (millis() - last_status_check > 5000) {
        last_status_check = millis();
        
        // MANUEL STATUS OKUMA (IRQ beklemeden!)
        uint32_t sys_status = dwm3000_read_sys_status();
        Serial.printf("\n[DEBUG] Manual Status Check: 0x%08X | IRQ Pin: %d\n", sys_status, digitalRead(DWM3000_IRQ_PIN));
        
        // Eğer herhangi bir RX event varsa göster
        if (sys_status & SYS_STATUS_RXPRD) {
            Serial.println("  ✓ Preamble detected!");
        }
        if (sys_status & SYS_STATUS_RXSFDD) {
            Serial.println("  ✓ SFD detected!");
        }
        if (sys_status & SYS_STATUS_RXPHD) {
            Serial.println("  ✓ PHY Header detected!");
        }
        if (sys_status & SYS_STATUS_RXDFR) {
            Serial.println("  ✓ Data Frame Ready!");
        }
        if (sys_status & SYS_STATUS_RXFCG) {
            Serial.println("  ★ FRAME RECEIVED!");
            
            // Frame var ama IRQ gelmemiş - Elle işle!
            uint8_t rx_buffer[127];
            dwm3000_read_reg(DWM3000_REG_RX_BUFFER, rx_buffer, 16);
            
            Serial.print("  Data: ");
            for (int i = 0; i < 16; i++) {
                Serial.printf("%02X ", rx_buffer[i]);
            }
            Serial.println();
            
            // Status temizle
            dwm3000_clear_sys_status(SYS_STATUS_RXFCG);
            frame_count++;
        }
        if (sys_status & SYS_STATUS_RXFCE) {
            Serial.println("  -> Frame CRC Error!");
        }
        if (sys_status & SYS_STATUS_RXPHE) {
            Serial.println("  -> PHY Header Error!");
        }
        
        if (sys_status == 0 || sys_status == 0x00000004) {
            Serial.println("  -> DWM3000 idle, waiting for signal...");
        }
    }
    
    // Her 10 saniyede bir istatistik yazdır
    static uint32_t last_stats_time = 0;
    if (millis() - last_stats_time > 10000) {
        last_stats_time = millis();
        
        Serial.println("\n========== STATISTICS ==========");
        Serial.printf("IRQ Triggers: %u\n", irq_count);
        Serial.printf("Frames Received: %u\n", frame_count);
        Serial.printf("CRC Errors: %u\n", error_count);
        Serial.printf("Success Rate: %.1f%%\n", 
                      frame_count > 0 ? (100.0 * frame_count / (frame_count + error_count)) : 0.0);
        Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
        Serial.printf("Free RAM: %d KB\n", ESP.getFreeHeap() / 1024);
        
        // DWM3000 Register Debug
        uint32_t sys_status = dwm3000_read_sys_status();
        Serial.printf("\n[DEBUG] SYS_STATUS: 0x%08X\n", sys_status);
        Serial.printf("  - RXFCG (Good Frame): %s\n", (sys_status & SYS_STATUS_RXFCG) ? "YES" : "NO");
        Serial.printf("  - RXFCE (CRC Error): %s\n", (sys_status & SYS_STATUS_RXFCE) ? "YES" : "NO");
        Serial.printf("  - RXPHE (PHY Error): %s\n", (sys_status & SYS_STATUS_RXPHE) ? "YES" : "NO");
        Serial.printf("  - RXFTO (Timeout): %s\n", (sys_status & SYS_STATUS_RXRFTO) ? "YES" : "NO");
        
        if (frame_count > 0) {
            uint32_t time_since_last = (millis() - last_frame_time) / 1000;
            Serial.printf("Last frame: %lu seconds ago\n", time_since_last);
            
            if (time_since_last > 5) {
                Serial.println("[WARNING] No frames received recently!");
                Serial.println("Check if K4W Tag is powered on and transmitting.");
            }
        } else {
            Serial.println("[INFO] No frames received yet.");
            Serial.println("Waiting for K4W Tag to transmit...");
        }
        
        Serial.println("================================\n");
    }
    
    delay(10);
}

/**
 * ============================================================================
 * KULLANIM NOTLARI:
 * ============================================================================
 * 
 * 1. DONANIM BAĞLANTISI:
 *    - DWM3000 modülünü Freenove breakout board üzerinden bağlayın
 *    - Tüm SPI pinlerini (SCLK, MOSI, MISO, CS) kontrol edin
 *    - EXTON pinini 3.3V'a bağlayın (güç kontrolü)
 *    - IRQ ve RST pinlerini bağlayın
 * 
 * 2. İLK TEST:
 *    - Serial Monitor'ü açın (115200 baud)
 *    - Device ID'nin başarıyla okunduğunu kontrol edin
 *    - "READY" mesajını görmelisiniz
 * 
 * 3. K4W TAG İLE TEST:
 *    - K4W Tag'ı açın (5 Hz Blink frame yayınlıyor)
 *    - ESP32-S3 Anchor frame'leri almalı
 *    - Her frame'de RGB LED yeşil yanacak
 *    - Serial Monitor'de frame bilgileri görünecek
 * 
 * 4. SORUN GİDERME:
 *    - Device ID 0x00000000 veya 0xFFFFFFFF → SPI bağlantı hatası
 *    - Frame alınmıyor → K4W Tag açık mı? Aynı kanalda mı?
 *    - CRC hataları → Sinyal zayıf, mesafe çok uzak
 * 
 * 5. SONRAKİ ADIMLAR:
 *    - Mesafe ölçümü (TWR - Two-Way Ranging)
 *    - MQTT entegrasyonu
 *    - Röle kontrolü
 *    - Flutter app bağlantısı
 * 
 * ============================================================================
 */
