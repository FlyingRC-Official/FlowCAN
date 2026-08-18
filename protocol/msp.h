#ifndef FLOWCAN_MSP_H
#define FLOWCAN_MSP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "flowcan/types.h"

#define MSP_MAX_PAYLOAD 64U

typedef struct {
    uint16_t command;
    uint16_t payload_length;
    uint8_t payload[MSP_MAX_PAYLOAD];
} msp_frame_t;

typedef enum {
    MSP_PARSE_IDLE = 0, MSP_PARSE_X, MSP_PARSE_DIRECTION, MSP_PARSE_FLAGS,
    MSP_PARSE_COMMAND_LO, MSP_PARSE_COMMAND_HI, MSP_PARSE_LENGTH_LO,
    MSP_PARSE_LENGTH_HI, MSP_PARSE_PAYLOAD, MSP_PARSE_CRC
} msp_parse_state_t;

typedef struct {
    msp_parse_state_t state;
    msp_frame_t frame;
    uint16_t offset;
    uint8_t crc;
} msp_parser_t;

uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t value);
size_t msp_encode_v2(uint16_t command, const uint8_t *payload, uint16_t length,
                     uint8_t *output, size_t capacity);
size_t msp_encode_flow(const flow_sample_t *sample, uint8_t *output, size_t capacity);
size_t msp_encode_range(const range_sample_t *sample, uint8_t *output, size_t capacity);
void msp_parser_init(msp_parser_t *parser);
bool msp_parser_consume(msp_parser_t *parser, uint8_t byte, msp_frame_t *frame);

#endif
