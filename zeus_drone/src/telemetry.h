#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#define USE_TELEMETRY
#define MAVLINK_TELEMETRY

#define TELEMETRY_PORT "/dev/ttyUSB0"

#define NO_GPS 0
#define NOT_ENOUGH_SATELLITES 1
#define GPS_OK 2


typedef struct __gps_info { 	// info from gps sensors	
	float lat, lon;
#ifdef MAVLINK_TELEMETRY
	float altitude;
	uint8_t satellites_visible;
	uint8_t vel; 		// GPS ground speed
	uint16_t cog; 		// Course over ground. NOT heading, but direction of movement in degree
	uint8_t fix_type;	// 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5:RTK
	uint64_t time_usec;	// Timestamp microseconds since UNIX epoch or microseconds since system boot	
#endif
} gps_info_t;

typedef struct __telemetry_info {
	gps_info_t gps;
} telemetry_info_t;

int initTelemetry();
void processTelemetry(telemetry_info_t *telemetry);
void freeTelemetry();

#endif
