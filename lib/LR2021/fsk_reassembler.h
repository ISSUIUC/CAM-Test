#pragma once

#include <stdint.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "fsk_framer_rx.h"

#define FSK_RX_FRAME_TIMEOUT_MS 150u

// Maximum JPEG frame size the reassembler will accept.
#define FSK_MAX_RX_FRAME_SIZE 2 * 720 * 480

#define FSK_RX_BYTE_FIELD_LENGTH 256u

struct FskReassemblyState
{
    uint8_t frame_id;         // frame_id currently being assembled
    uint8_t last_complete_id; // frame_id of the last frame handed off (0xFF = none)
    bool active;              // true while assembling a frame

    // Size / progress
    uint32_t total_size;     // original payload length (from first fragment header)
    uint16_t total_frags;    // derived: fsk_frame_count(total_size)
    uint16_t frags_received; // unique fragments stored so far
    uint32_t completed_size; // total_size saved before reset — read this at frame_ready

    // Timing
    uint32_t last_frag_ms; // millis() of last successfully stored fragment

    // Received-fragment bitfield (one bit per fragment index)
    uint8_t recv_bits[FSK_RX_BYTE_FIELD_LENGTH];

    // Output buffer (allocated once in SPIRAM by fsk_reassembler_init)
    uint8_t *data_buf;
    uint32_t data_buf_len;

    // Debug counters
    uint32_t stat_crc_drop;     // packets dropped due to LR2021_ERR_CRC_MISMATCH
    uint32_t stat_frame_resets; // times a new frame_id interrupted an in-progress frame
    uint32_t stat_frags_total;  // total fragments successfully written across all frames
};

enum FskIngestResult : uint8_t
{
    FSK_INGEST_OK = 0,       // fragment stored, frame still in progress
    FSK_INGEST_COMPLETE = 1, // all fragments received — frame ready in data_buf
    FSK_INGEST_TIMEOUT = 2,  // partial frame flushed after stale timeout
    FSK_INGEST_DUP = 3,      // duplicate fragment index, ignored
    FSK_INGEST_OVERFLOW = 4, // fragment write would overflow data_buf
    FSK_INGEST_BAD_HDR = 5,  // header parse failed or sanity check failed
};

static inline bool fsk_reassembler_init(FskReassemblyState *s, uint32_t buf_size)
{
    memset(s, 0, sizeof(*s));
    s->last_complete_id = 0xFF;
    s->data_buf = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    if (!s->data_buf)
        return false;
    s->data_buf_len = buf_size;
    return true;
}

// Resets per-frame fields without touching the buffer or debug counters.
static inline void fsk_reassembler_reset(FskReassemblyState *s)
{
    s->active = false;
    s->frame_id = 0;
    s->total_size = 0;
    s->total_frags = 0;
    s->frags_received = 0;
    s->last_frag_ms = 0;
    memset(s->recv_bits, 0, FSK_RX_BYTE_FIELD_LENGTH);
}

// Returns true if fragment index was already received.
static inline bool fsk_fragment_seen(const FskReassemblyState *s, uint16_t idx)
{
    uint16_t byte_idx = idx / 8;
    uint8_t bit_mask = 1u << (idx % 8);
    if (byte_idx >= FSK_RX_BYTE_FIELD_LENGTH)
        return false;
    return (s->recv_bits[byte_idx] & bit_mask) != 0;
}

// Marks fragment index as received.
static inline void fsk_fragment_mark(FskReassemblyState *s, uint16_t idx)
{
    uint16_t byte_idx = idx / 8;
    uint8_t bit_mask = 1u << (idx % 8);
    if (byte_idx < FSK_RX_BYTE_FIELD_LENGTH)
        s->recv_bits[byte_idx] |= bit_mask;
}

static inline FskIngestResult fsk_reassembler_ingest(
    FskReassemblyState *s,
    const uint8_t *pkt,
    uint32_t now_ms)
{
    uint8_t incoming_id;
    uint16_t frag_index;
    uint32_t total_size;

    if (!fsk_parse_header(pkt, &incoming_id, &frag_index, &total_size))
        return FSK_INGEST_BAD_HDR;

    if (total_size == 0 || total_size > s->data_buf_len)
        return FSK_INGEST_BAD_HDR;

    uint16_t total_frags = fsk_frame_count(total_size);

    // New frame ID — start fresh
    if (!s->active || incoming_id != s->frame_id)
    {
        if (s->active)
            s->stat_frame_resets++;
        fsk_reassembler_reset(s);
        s->active = true;
        s->frame_id = incoming_id;
        s->total_size = total_size;
        s->total_frags = total_frags;
    }

    if (frag_index >= s->total_frags)
        return FSK_INGEST_BAD_HDR;

    if (fsk_fragment_seen(s, frag_index))
        return FSK_INGEST_DUP;

    if (!fsk_reassemble_fragment(pkt, frag_index, total_size,
                                 s->data_buf, s->data_buf_len))
        return FSK_INGEST_OVERFLOW;

    fsk_fragment_mark(s, frag_index);
    s->frags_received++;
    s->last_frag_ms = now_ms;
    s->stat_frags_total++;

    // All fragments arrived
    if (s->frags_received >= s->total_frags)
    {
        s->last_complete_id = s->frame_id;
        s->completed_size = s->total_size; // save before reset clears it
        fsk_reassembler_reset(s);
        return FSK_INGEST_COMPLETE;
    }

    return FSK_INGEST_OK;
}

static inline FskIngestResult fsk_reassembler_check_timeout(
    FskReassemblyState *s,
    uint32_t now_ms)
{
    if (!s->active || s->frags_received == 0)
        return FSK_INGEST_OK;

    if (now_ms - s->last_frag_ms >= FSK_RX_FRAME_TIMEOUT_MS)
    {
        s->last_complete_id = s->frame_id;
        s->completed_size = s->total_size; // save before reset clears it
        fsk_reassembler_reset(s);
        return FSK_INGEST_TIMEOUT;
    }

    return FSK_INGEST_OK;
}