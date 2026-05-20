#pragma once

#include <stdint.h>
#include "fsk_framer.h"

static inline bool fsk_parse_header(
    const uint8_t *pkt,
    uint8_t *out_frame_id,
    uint16_t *out_frag_index,
    uint32_t *out_total_size)
{
    if (!pkt)
        return false;
    *out_frame_id = pkt[0];
    *out_frag_index = ((uint16_t)pkt[1] << 8) | pkt[2];
    *out_total_size = ((uint32_t)pkt[3] << 24) | ((uint32_t)pkt[4] << 16) | ((uint32_t)pkt[5] << 8) | (uint32_t)pkt[6];
    return true;
}

static inline bool fsk_reassemble_fragment(
    const uint8_t *pkt,
    uint16_t frag_index,
    uint32_t total_size,
    uint8_t *dst_buf,
    uint32_t dst_buf_len)
{
    if (!pkt || !dst_buf)
        return false;

    uint32_t offset = (uint32_t)frag_index * FSK_FRAG_DATA_SIZE;
    uint32_t remaining = (offset < total_size) ? (total_size - offset) : 0u;
    uint16_t copy_len = (remaining > FSK_FRAG_DATA_SIZE)
                            ? (uint16_t)FSK_FRAG_DATA_SIZE
                            : (uint16_t)remaining;

    if (copy_len == 0 || offset + copy_len > dst_buf_len)
        return false;

    memcpy(dst_buf + offset, pkt + FSK_FRAG_HDR_SIZE, copy_len);
    return true;
}