#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdint.h>

typedef enum gps_parser_state_e {
    GPS_PARSER_STATE_IDLE = 0,
    GPS_PARSER_STATE_PREFIX,
    GPS_PARSER_STATE_DATA,
    GPS_PARSER_STATE_CHECKSUM,
}gps_parser_state_t;

typedef enum gps_parser_result_e {
    GPS_PARSER_RESULT_INCOMPLETE = 0 ,
    GPS_PARSER_RESULT_COMPLETE,
    GPS_PARSER_RESULT_CHECKSUM_ERR,
    GPS_PARSER_RESULT_ERR,
}gps_parser_result_t;

typedef struct
{
    gps_parser_state_t state;
    gps_parser_result_t result;
    
    uint8_t field_cnt; // delimiter 체크 개수
}gps_parser_t;

#endif