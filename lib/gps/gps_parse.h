#ifndef GPS_PARSE_H
#define GPS_PARSE_H

#include "gps_types.h"
#include "gps.h"
#include "parser.h"
#include <stdint.h>

/* 숫자, 문자 파싱 */
int32_t gps_parse_number(gps_t *gps);
double gps_parse_double(gps_t *gps);
float gps_parse_float(gps_t *gps);
char gps_parse_character(gps_t *gps);

#endif
