#pragma once

#include <RadioLib.h>
#include <Arduino.h>
#include "../../src/pins.h"

static const uint16_t PAYLOAD_SIZE_FLRC = 511;
static const uint8_t PAYLOAD_SIZE_FSK = RADIOLIB_LR2021_MAX_PACKET_LENGTH;

#define SPI_SPEED 16000000

static uint8_t SYNC_WORD_FLRC[] = {0x2D, 0x01, 0x4B, 0x1D};
static uint8_t SYNC_WORD_FSK[] = {0x01, 0x23, 0x45, 0x67,
                                  0x89, 0xAB, 0xCD, 0xEF};
static const char *CALL_SIGN = "KE2CNQ";
static const int IRQ_PIN = 9;

#define FREQ_915 915.0 // mhz
#define FREQ_434 434.0 // mhz

#define FREQ FREQ_434     // MHz
#define FREQ_FSK FREQ_434 // MHz
#define BITRATE_FLRC 2600 // Kbps -> reference table 18-1 on datahseet

#define BITRATE_FSK_434 1000
#define RX_BANDWIDTH_FSK_434 2222.22 // khz Table 11-2
#define FREQ_DEV_FSK_434 250.0       // khz

#define BITRATE_FSK_915 2000
#define RX_BANDWIDTH_FSK_915 1111.10 // khz Table 11-2
#define FREQ_DEV_FSK_915 500         // khz

#define POWER 22           // dBm Tx Power
#define PREAMBLE_LENGTH 24 // bit
#define XTAL_MODE 0        // tcxoVoltage argument, setting to 0 brings it to xtal mode
#define CRC_LENGTH 3       // bits, 0, 1, 2, 3

/* FIFO */
#define FIFO_TX_CHUNK_1 255    // bytes in first write (fills FIFO without overflow)
#define FIFO_TX_CHUNK_2 256    // bytes in second write (remainder of 511)
#define FIFO_TX_LOW_THRESH 128 // fire TxFifoLow IRQ when FIFO drops below this level

#define FIFO_RX_HIGH_THRESH 200 // fire RxFifo IRQ when RX FIFO fills above this

/* Error stuff */
#define LR2021_ERR_NONE 0
#define LR2021_ERR_GENERIC -6767
#define LR2021_ERR_SPI_INIT_FAILED -2001
#define LR2021_ERR_FLRC_INIT_FAILED -2002
#define LR2021_ERR_FSK_INIT_FAILED -2009
#define LR2021_ERR_SYNC_WORD_FAILED -2003
#define LR2021_ERR_CRC_CONFIG_FAILED -2004
#define LR2021_ERR_PKT_LEN_FAILED -2005
#define LR2021_ERR_TX_TIMEOUT -2006
#define LR2021_ERR_RX_TIMEOUT -2007
#define LR2021_ERR_CRC_MISMATCH -2008
#define LR2021_ERR_ADDRESS_FILTERING -2010
#define LR2021_ERR_DATASHAPING -2011
#define LR2021_ERR_FSK_FIXED_PACKET_MD -2012

struct LR2021Error
{
    int driverCode;
    int radioLibCode;

    bool ok() const { return driverCode == LR2021_ERR_NONE; }

    const char *stageStr() const
    {
        switch (driverCode)
        {
        case LR2021_ERR_NONE:
            return "OK";
        case LR2021_ERR_SPI_INIT_FAILED:
            return "SPI bus init failed";
        case LR2021_ERR_FSK_INIT_FAILED:
            return "radio.beginFSK failed";
        case LR2021_ERR_FLRC_INIT_FAILED:
            return "radio.beginFLRC() failed";
        case LR2021_ERR_SYNC_WORD_FAILED:
            return "radio.setSyncWord() failed";
        case LR2021_ERR_CRC_CONFIG_FAILED:
            return "radio.setCRC() failed";
        case LR2021_ERR_PKT_LEN_FAILED:
            return "radio.fixedPacketLengthMode() failed";
        case LR2021_ERR_TX_TIMEOUT:
            return "lr2021.transmit() tx timeout";
        case LR2021_ERR_RX_TIMEOUT:
            return "lr2021.receive() rx timeout";
        case LR2021_ERR_CRC_MISMATCH:
            return "lr2021.receive() crc mismatch";
        case LR2021_ERR_ADDRESS_FILTERING:
            return "radio.disableAddressFiltering(); failed";
        case LR2021_ERR_DATASHAPING:
            return "radio.setDataShaping(); failed";
        case LR2021_ERR_FSK_FIXED_PACKET_MD:
            return "radio.fixedPacketLengthMode(PAYLOAD_SIZE_FSK); failed";
        case LR2021_ERR_GENERIC:
            return "GL bro idk where this error is";
        default:
            return "unknown error";
        }
    }
};
