/**
 * DWM3000 Register Definitions
 * 
 * Based on Qorvo DW3000 User Manual
 * Compatible with DWM3000 module (DW3110 chip)
 */

#ifndef DWM3000_REGS_H
#define DWM3000_REGS_H

#include <stdint.h>

// ============================================================================
// REGISTER ADDRESSES
// ============================================================================

// Device ID and System Registers
#define DWM3000_REG_DEV_ID          0x00    // Device ID (Read-only)
#define DWM3000_REG_SYS_CFG         0x04    // System Configuration
#define DWM3000_REG_SYS_STATUS      0x08    // System Event Status
#define DWM3000_REG_SYS_MASK        0x0E    // System Event Mask
#define DWM3000_REG_SYS_CTRL        0x0D    // System Control
#define DWM3000_REG_SYS_ENABLE      0x0E    // System Event Enable

// RX Registers
#define DWM3000_REG_RX_FINFO        0x10    // RX Frame Information
#define DWM3000_REG_RX_BUFFER       0x11    // RX Frame Buffer
#define DWM3000_REG_RX_FQUAL        0x12    // RX Frame Quality
#define DWM3000_REG_RX_TIME         0x13    // RX Time Stamp

// TX Registers
#define DWM3000_REG_TX_BUFFER       0x14    // TX Frame Buffer
#define DWM3000_REG_TX_FCTRL        0x15    // TX Frame Control
#define DWM3000_REG_TX_TIME         0x17    // TX Time Stamp

// Timing Registers
#define DWM3000_REG_DX_TIME         0x0A    // Delayed TX/RX Time
#define DWM3000_REG_SYS_TIME        0x06    // System Time Counter

// Configuration Registers
#define DWM3000_REG_CHAN_CTRL       0x1F    // Channel Control
#define DWM3000_REG_TX_POWER        0x1E    // TX Power Control
#define DWM3000_REG_RF_CONF         0x28    // RF Configuration

// OTP (One-Time Programmable) Memory
#define DWM3000_REG_OTP_IF          0x2D    // OTP Interface
#define DWM3000_REG_OTP_ADDR        0x2E    // OTP Address
#define DWM3000_REG_OTP_CTRL        0x2F    // OTP Control
#define DWM3000_REG_OTP_RDAT        0x31    // OTP Read Data

// ============================================================================
// DEVICE ID VALUES
// ============================================================================

#define DWM3000_DEVICE_ID           0xDECA0302  // Expected Device ID for DW3110
#define DWM3000_DEVICE_ID_ALT       0xDECA0301  // Alternative Device ID

// ============================================================================
// SYSTEM CONTROL REGISTER (0x0D) - Commands
// ============================================================================

#define SYS_CTRL_TXSTRT             0x00000001  // Transmit Start
#define SYS_CTRL_TXDLYS             0x00000002  // Transmit Delayed Start
#define SYS_CTRL_TRXOFF             0x00000040  // Transceiver Off
#define SYS_CTRL_WAIT4RESP          0x00000080  // Wait for Response
#define SYS_CTRL_RXENAB             0x00000100  // Receiver Enable
#define SYS_CTRL_RXDLYE             0x00000200  // Receiver Delayed Enable

// ============================================================================
// SYSTEM STATUS REGISTER (0x08) - Event Flags
// ============================================================================

#define SYS_STATUS_IRQS             0x00000001  // Interrupt Request Status
#define SYS_STATUS_CPLOCK           0x00000002  // Clock PLL Lock
#define SYS_STATUS_ESYNCR           0x00000004  // External Sync Clock Reset
#define SYS_STATUS_AAT              0x00000008  // Automatic Acknowledge Trigger
#define SYS_STATUS_TXFRB            0x00000010  // Transmit Frame Begins
#define SYS_STATUS_TXPRS            0x00000020  // Transmit Preamble Sent
#define SYS_STATUS_TXPHS            0x00000040  // Transmit PHY Header Sent
#define SYS_STATUS_TXFRS            0x00000080  // Transmit Frame Sent
#define SYS_STATUS_RXPRD            0x00000100  // Receiver Preamble Detected
#define SYS_STATUS_RXSFDD           0x00000200  // Receiver SFD Detected
#define SYS_STATUS_RXPHD            0x00000400  // Receiver PHY Header Detect
#define SYS_STATUS_RXPHE            0x00000800  // Receiver PHY Header Error
#define SYS_STATUS_RXDFR            0x00001000  // Receiver Data Frame Ready
#define SYS_STATUS_RXFCG            0x00002000  // Receiver FCS Good
#define SYS_STATUS_RXFCE            0x00004000  // Receiver FCS Error
#define SYS_STATUS_RXRFSL           0x00008000  // Receiver Reed Solomon Frame Sync Loss
#define SYS_STATUS_RXRFTO           0x00010000  // Receiver Frame Wait Timeout
#define SYS_STATUS_RXOVRR           0x00020000  // Receiver Overrun
#define SYS_STATUS_RXPTO            0x00200000  // Preamble Detection Timeout
#define SYS_STATUS_RXSFDTO          0x04000000  // Receive SFD Timeout
#define SYS_STATUS_RXRSCS           0x10000000  // Receiver Reed-Solomon Correction Status

// ============================================================================
// SYSTEM CONFIGURATION REGISTER (0x04)
// ============================================================================

#define SYS_CFG_FFEN                0x00000001  // Frame Filtering Enable
#define SYS_CFG_FFBC                0x00000002  // Frame Filtering Behave as Coordinator
#define SYS_CFG_FFAB                0x00000004  // Frame Filtering Allow Beacon
#define SYS_CFG_FFAD                0x00000008  // Frame Filtering Allow Data
#define SYS_CFG_FFAA                0x00000010  // Frame Filtering Allow Acknowledgment
#define SYS_CFG_FFAM                0x00000020  // Frame Filtering Allow MAC Command
#define SYS_CFG_RXAUTR              0x20000000  // Receiver Auto-Re-Enable
#define SYS_CFG_AUTOACK             0x40000000  // Automatic Acknowledgement Enable

// ============================================================================
// CHANNEL CONTROL REGISTER (0x1F)
// ============================================================================

#define CHAN_CTRL_TX_CHAN_MASK      0x0000000F  // TX Channel Mask
#define CHAN_CTRL_RX_CHAN_MASK      0x000000F0  // RX Channel Mask
#define CHAN_CTRL_DWSFD             0x00020000  // Decawave SFD Enable
#define CHAN_CTRL_RXPRF_MASK        0x000C0000  // RX PRF Mask
#define CHAN_CTRL_TNSSFD            0x00100000  // Non-standard SFD in TX
#define CHAN_CTRL_RNSSFD            0x00200000  // Non-standard SFD in RX

// Channel numbers
#define CHANNEL_5                   5           // Channel 5 (6.5 GHz)
#define CHANNEL_9                   9           // Channel 9 (8 GHz)

// PRF (Pulse Repetition Frequency)
#define PRF_16M                     1           // 16 MHz PRF
#define PRF_64M                     2           // 64 MHz PRF

// ============================================================================
// RX FRAME INFORMATION REGISTER (0x10)
// ============================================================================

#define RX_FINFO_RXFLEN_MASK        0x0000007F  // Receive Frame Length (0-127)
#define RX_FINFO_RXNSPL_MASK        0x00001800  // Receive Non-Standard Preamble Length
#define RX_FINFO_RXPSR_MASK         0x000C0000  // RX Preamble Repetition
#define RX_FINFO_RXPACC_MASK        0xFFF00000  // Preamble Accumulation Count

// ============================================================================
// SPI COMMANDS
// ============================================================================

#define SPI_READ_CMD                0x00        // SPI Read command (bit 7 = 0)
#define SPI_WRITE_CMD               0x80        // SPI Write command (bit 7 = 1)
#define SPI_MASK_CMD                0x7F        // Mask for register address

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

#define DWM3000_RESET_DELAY_MS      10          // Reset pulse duration
#define DWM3000_INIT_DELAY_MS       100         // Initialization delay after reset
#define DWM3000_SPI_SPEED_SLOW      2000000     // 2 MHz (initialization)
#define DWM3000_SPI_SPEED_FAST      20000000    // 20 MHz (normal operation)

// ============================================================================
// FRAME TYPES (IEEE 802.15.4)
// ============================================================================

#define FRAME_TYPE_BEACON           0x00
#define FRAME_TYPE_DATA             0x01
#define FRAME_TYPE_ACK              0x02
#define FRAME_TYPE_MAC_CMD          0x03
#define FRAME_TYPE_BLINK            0x05        // K4W Tag uses Blink frames

// ============================================================================
// HELPER MACROS
// ============================================================================

#define DWM3000_READ_REG(addr)      ((addr) & SPI_MASK_CMD)
#define DWM3000_WRITE_REG(addr)     (((addr) & SPI_MASK_CMD) | SPI_WRITE_CMD)

#endif // DWM3000_REGS_H
