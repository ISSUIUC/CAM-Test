/* DEPCREATED */

#pragma once

#include <RadioLib.h>
#include <Arduino.h>
#include "radio.h"

#ifndef LR2021_FLRC_DRIVER
#define LR2021_FLRC_DRIVER
// Based off table 18-9
typedef struct
{
    uint16_t packet_length_bytes; // pkt_len[15:0]
    int16_t rssi_avg_in_dbm;      // -rssi_avg / 2 dBm (integer part)
    int16_t rssi_sync_in_dbm;     // -rssi_sync / 2 dBm (integer part)
    uint8_t rssi_avg_half_dbm;    // rssi_avg(0) — bit 2 of byte 6, adds 0.5 dBm
    uint8_t rssi_sync_half_dbm;   // rssi_sync(0) — bit 0 of byte 6, adds 0.5 dBm
    uint8_t syncword_index;       // sw_num[3:0] — bits [7:4] of byte 6
} LR2021FlrcPktStatus;

// BAD BAD BAD BAD ASFJLADSFJ;LASDJFKASDLFJKLAS DO NOT USE PLEASE
class LR2021FLRCDriver
{
private:
    SPIClass mySPI;
    SPISettings spiSettings;

    static LR2021FLRCDriver *_instance;

    /* Flags */
    volatile bool radioEvent = false;
    static void setFlag();

    /* Tx stuff */
    uint8_t *fifoRefillPtr;
    uint16_t fifoRefillLen;

    void txFIFOWriteChunkOne(uint8_t *data);
    void txSet();
    void txFIFOWriteChunkTwo();

    /* Rx stuff */
    uint16_t rxBytesRead; // running count of bytes drained mid-packet
    bool rxFifoHighFired;

    void rxSet();
    void rxFIFODrainChunk(uint8_t *dst, uint16_t len);

    uint32_t readIRQ();
    void clearFifoIrq(uint8_t rxFlags, uint8_t txFlags);

    /* SPI */
    void spiWrite(const uint8_t *cmd, size_t len);
    void spiTransfer(const uint8_t *txBuf, uint8_t *rxBuf, size_t len);

public:
    LR2021 radio;

    LR2021FLRCDriver();
    LR2021Error init();
    LR2021Error setFIFO();
    LR2021Error setIRQ();
    LR2021Error transmit(uint8_t *data);
    LR2021Error receive(uint8_t *data, uint16_t len, LR2021FlrcPktStatus *pkt_status = nullptr);

    bool rxGetFLRCPcktStatus(LR2021FlrcPktStatus *pkt_status);

    void transmitCallSign();
};

#endif