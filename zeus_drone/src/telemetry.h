#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdatomic.h>

#define USE_TELEMETRY
#define MAVLINK_TELEMETRY

#define TELEMETRY_PORT "/dev/ttyUSB0"

#define NO_GPS 0
#define NOT_ENOUGH_SATELLITES 1
#define GPS_OK 2


typedef struct __gps_info { 	// info from gps sensors	
	float lat, lon;
	float altitude;
	float vel; 		// GPS ground speed
	float cog; 		// Course over ground. NOT heading, but direction of movement in degree
	uint8_t satellites_visible;
	uint8_t fix_type;	// 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5:RTK
	uint64_t time_usec;	// Timestamp microseconds since UNIX epoch or microseconds since system boot	
} gps_info_t;


typedef struct __sys_status {
	int8_t battery_remaining; // 0-100%, -1 if unknown
} sys_status;

typedef struct __system_attitude {
	uint32_t time_boot_ms;
	float roll;
	float pitch;
	float yaw;
	float rollspeed;
	float yawspeed;
	float pitchspeed;
} system_attitude;

typedef struct __rc_channels {
	uint32_t time_boot_ms;
	uint16_t chan1_raw;
	uint16_t chan2_raw;
	uint16_t chan3_raw;
	uint16_t chan4_raw;
	uint16_t chan5_raw;
	uint16_t chan6_raw;
	uint16_t chan7_raw;
	uint16_t chan8_raw;
} rc_channels;

typedef struct __global_pos {
	float lat;
	float lon;
	float alt;
	float relative_alt;
	float vx;
	float vy;
	float vz;
	float hdg;
	uint32_t time_boot_ms;
} global_pos;


typedef struct __vfr_hud {
	float airspeed;
	float groundspeed;
	float alt;
	float climb;
	int16_t heading;
	uint16_t throttle;
} vfr_h;

/*	This is our telemetry struct. Before accessing it we must acquire the lock!!!!	*/
typedef struct __telemetry_info {
	gps_info_t *gps;
	system_attitude *attitude;
	global_pos *global_pos;
	sys_status *status;
	rc_channels *rc_data;
	vfr_h *vfr;
	atomic_flag lock;
} telemetry_info_t;

int initTelemetry(telemetry_info_t *telemetry);
void processTelemetry(telemetry_info_t *telemetry);
void freeTelemetry(telemetry_info_t *telemetry);

#endif