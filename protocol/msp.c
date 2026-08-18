#include "msp.h"
#include "config.h"
#include <string.h>

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t value)
{
    crc ^= value;
    for (unsigned i = 0; i < 8U; i++) {
        const uint8_t shifted = (uint8_t)(crc << 1U);
        crc = (uint8_t)((crc & 0x80U) ? (uint8_t)(shifted ^ 0xD5U) : shifted);
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8U); }
static void put_i32(uint8_t *p, int32_t v)
{
    const uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)u; p[1] = (uint8_t)(u >> 8U); p[2] = (uint8_t)(u >> 16U); p[3] = (uint8_t)(u >> 24U);
}

size_t msp_encode_v2(uint16_t command, const uint8_t *payload, uint16_t length,
                     uint8_t *output, size_t capacity)
{
    const size_t total = (size_t)length + 9U;
    if ((output == NULL) || (payload == NULL && length != 0U) || length > MSP_MAX_PAYLOAD || capacity < total) return 0U;
    output[0] = '$'; output[1] = 'X'; output[2] = '<'; output[3] = 0U;
    put_u16(&output[4], command); put_u16(&output[6], length);
    if (length != 0U) memcpy(&output[8], payload, length);
    uint8_t crc = 0U;
    for (size_t i = 3U; i < 8U + length; i++) crc = msp_crc8_dvb_s2(crc, output[i]);
    output[8U + length] = crc;
    return total;
}

size_t msp_encode_flow(const flow_sample_t *sample, uint8_t *output, size_t capacity)
{
    uint8_t payload[9];
    payload[0] = sample->valid ? sample->quality : 0U;
    put_i32(&payload[1], sample->valid ? sample->count_x : 0);
    put_i32(&payload[5], sample->valid ? sample->count_y : 0);
    return msp_encode_v2(MSP2_SENSOR_OPTIC_FLOW, payload, sizeof(payload), output, capacity);
}

size_t msp_encode_range(const range_sample_t *sample, uint8_t *output, size_t capacity)
{
    uint8_t payload[5];
    const bool valid = sample->status == RANGE_STATUS_VALID;
    payload[0] = valid ? sample->quality : 0U;
    put_i32(&payload[1], valid ? sample->distance_mm : -1);
    return msp_encode_v2(MSP2_SENSOR_RANGEFINDER, payload, sizeof(payload), output, capacity);
}

void msp_parser_init(msp_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void parser_reset(msp_parser_t *p) { p->state = MSP_PARSE_IDLE; p->offset = 0U; p->crc = 0U; }

bool msp_parser_consume(msp_parser_t *p, uint8_t b, msp_frame_t *out)
{
    switch (p->state) {
    case MSP_PARSE_IDLE: p->state = (b == '$') ? MSP_PARSE_X : MSP_PARSE_IDLE; break;
    case MSP_PARSE_X: p->state = (b == 'X') ? MSP_PARSE_DIRECTION : MSP_PARSE_IDLE; break;
    case MSP_PARSE_DIRECTION: p->state = (b == '<') ? MSP_PARSE_FLAGS : MSP_PARSE_IDLE; break;
    case MSP_PARSE_FLAGS: p->crc = msp_crc8_dvb_s2(0U, b); p->state = MSP_PARSE_COMMAND_LO; break;
    case MSP_PARSE_COMMAND_LO: p->frame.command = b; p->crc = msp_crc8_dvb_s2(p->crc, b); p->state = MSP_PARSE_COMMAND_HI; break;
    case MSP_PARSE_COMMAND_HI: p->frame.command |= (uint16_t)b << 8U; p->crc = msp_crc8_dvb_s2(p->crc, b); p->state = MSP_PARSE_LENGTH_LO; break;
    case MSP_PARSE_LENGTH_LO: p->frame.payload_length = b; p->crc = msp_crc8_dvb_s2(p->crc, b); p->state = MSP_PARSE_LENGTH_HI; break;
    case MSP_PARSE_LENGTH_HI:
        p->frame.payload_length |= (uint16_t)b << 8U; p->crc = msp_crc8_dvb_s2(p->crc, b);
        if (p->frame.payload_length > MSP_MAX_PAYLOAD) parser_reset(p);
        else p->state = p->frame.payload_length == 0U ? MSP_PARSE_CRC : MSP_PARSE_PAYLOAD;
        break;
    case MSP_PARSE_PAYLOAD:
        p->frame.payload[p->offset++] = b; p->crc = msp_crc8_dvb_s2(p->crc, b);
        if (p->offset == p->frame.payload_length) p->state = MSP_PARSE_CRC;
        break;
    case MSP_PARSE_CRC:
        if (b == p->crc) { *out = p->frame; parser_reset(p); return true; }
        parser_reset(p); break;
    default: parser_reset(p); break;
    }
    return false;
}
