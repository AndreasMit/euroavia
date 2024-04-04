#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "asm/termbits.h"
#include <stdbool.h>
#include "telemetry.h"
#include "mavlink_layer.h"
#include <mavlink/common/mavlink.h>
#include <stdatomic.h>


#ifndef TELEMETRY_PORT
	#error "Telemetry port is not defined in mavlink_layer.c"
#endif

#ifndef MAVLINK_BAUD_RATE
	#error "MAVLINK baud rate is not defined"
#endif

static int mavlink_fd;

mavlink_status_t status;
mavlink_message_t msg;

const int chan = MAVLINK_COMM_0;

mavlink_global_position_int_t global_position;
mavlink_gps_raw_int_t gps_raw;
mavlink_battery_status_t battery_status;
mavlink_attitude_t attitude;
mavlink_heartbeat_t heartbeat;
mavlink_vfr_hud_t vfr_hud;
mavlink_rc_channels_raw_t rc_channels_raw;




void setTelemetryGPS(telemetry_info_t *telemetry) {

	// Acquire spin lock
	while (atomic_flag_test_and_set(&telemetry->lock));

	telemetry->gps->altitude = (float)gps_raw.alt / 1000;
	telemetry->gps->lat = (float)gps_raw.lat / 10000000;
	telemetry->gps->lon = (float)gps_raw.lon / 10000000;
	telemetry->gps->vel = (float)gps_raw.vel / 100;
	telemetry->gps->cog = (float)gps_raw.cog / 100;
	telemetry->gps->fix_type = gps_raw.fix_type;
	telemetry->gps->satellites_visible = gps_raw.satellites_visible;
	telemetry->gps->time_usec = gps_raw.time_usec;

	// release the lock
	atomic_flag_clear(&telemetry->lock);
}


void setTelemetryAttitude(telemetry_info_t *telemetry) {
	
	// Acquire spin lock
	while (atomic_flag_test_and_set(&telemetry->lock));

	telemetry->attitude->roll = attitude.roll;
	telemetry->attitude->pitch = attitude.pitch;
	telemetry->attitude->yaw = attitude.yaw;
	telemetry->attitude->rollspeed = attitude.rollspeed;
	telemetry->attitude->yawspeed = attitude.yawspeed;
	telemetry->attitude->pitchspeed = attitude.pitchspeed;

	// release the lock
	atomic_flag_clear(&telemetry->lock);

}

void setTelemetryGlobalPosition(telemetry_info_t *telemetry) {

	// Acquire spin lock
	while (atomic_flag_test_and_set(&telemetry->lock));

	telemetry->global_pos->lat = (float)global_position.lat / 10000000;
	telemetry->global_pos->lon = (float)global_position.lon / 10000000;
	telemetry->global_pos->alt = (float)global_position.alt / 1000;
	telemetry->global_pos->relative_alt = (float)global_position.relative_alt / 1000;
	telemetry->global_pos->vx = (float)global_position.vx / 100;
	telemetry->global_pos->vy = (float)global_position.vy / 100;
	telemetry->global_pos->vz = (float)global_position.vz / 100;

	// release the lock
	atomic_flag_clear(&telemetry->lock);
}


void setTelemetryBatteryStatus(telemetry_info_t *telemetry) {

	// Acquire spin lock
	while (atomic_flag_test_and_set(&telemetry->lock));

	telemetry->status->battery_remaining = battery_status.battery_remaining;

	// release the lock
	atomic_flag_clear(&telemetry->lock);
}


void setTelemetryRCRaw(telemetry_info_t *telemetry) {

	// Acquire spin lock
	while (atomic_flag_test_and_set(&telemetry->lock));

	telemetry->rc_data->chan1_raw = rc_channels_raw.chan1_raw;
	telemetry->rc_data->chan2_raw = rc_channels_raw.chan2_raw;
	telemetry->rc_data->chan3_raw = rc_channels_raw.chan3_raw;
	telemetry->rc_data->chan4_raw = rc_channels_raw.chan4_raw;
	telemetry->rc_data->chan5_raw = rc_channels_raw.chan5_raw;
	telemetry->rc_data->chan6_raw = rc_channels_raw.chan6_raw;
	telemetry->rc_data->chan7_raw = rc_channels_raw.chan7_raw;
	telemetry->rc_data->chan8_raw = rc_channels_raw.chan8_raw;

	// release the lock
	atomic_flag_clear(&telemetry->lock);
}


void setTelemetryVFR(telemetry_info_t *telemetry) {

	// Acquire spin lock
	while (atomic_flag_test_and_set(&telemetry->lock));

	telemetry->vfr->airspeed = vfr_hud.airspeed;
	telemetry->vfr->climb = vfr_hud.climb;
	telemetry->vfr->groundspeed = vfr_hud.groundspeed;
	telemetry->vfr->alt = vfr_hud.alt;
	telemetry->vfr->throttle = vfr_hud.throttle;
	telemetry->vfr->heading = vfr_hud.heading;

	// release the lock
	atomic_flag_clear(&telemetry->lock);
}




/*	Parse 1 byte at a time	*/
void handleTelemetry(telemetry_info_t *telemetry) {
	/* Read 1 byte from serial port */
	uint8_t byte;
	while (read(mavlink_fd, &byte, sizeof(byte))) {
		fprintf(stdout, "Byte read: %d\n", byte);
		
		if (mavlink_parse_char(chan, byte, &msg, &status)) {
			fprintf(stdout, "Received message with ID %u, sequence: %u from compontent %u of system %u\n", msg.msgid, msg.seq, msg.sysid);
			
			/* Payload decoding */
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
					mavlink_msg_global_position_int_decode(&msg, &global_position);
					fprintf(stdout, "MESSAGE GLOBAL POSITION\n");
					setTelemetryGlobalPosition(telemetry);
				}
					break;
				case MAVLINK_MSG_ID_GPS_RAW_INT: {
					mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
					fprintf(stdout, "MESSAGE GPS RAW\n");
					setTelemetryGPS(telemetry);
				}
					break;
				case MAVLINK_MSG_ID_BATTERY_STATUS: {
					mavlink_msg_battery_status_decode(&msg, &battery_status);
					fprintf(stdout, "MESSAGE BATTERY STATUS\n");
					setTelemetryBatteryStatus(telemetry);
				}
					break;
				case MAVLINK_MSG_ID_ATTITUDE: {
					mavlink_msg_attitude_decode(&msg, &attitude);
					fprintf(stdout, "MESSAGE ATTITUDE RECEIVED\n");
					setTelemetryAttitude(telemetry);
				}
					break;
				case MAVLINK_MSG_ID_HEARTBEAT: {
					mavlink_msg_heartbeat_decode(&msg, &heartbeat);
					fprintf(stdout, "HEARTBEAT RECEIVED\n");
				}	
					break;
				case MAVLINK_MSG_ID_VFR_HUD: {
					mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
					fprintf(stdout, "VFR HUD RECEIVED\n");
					setTelemetryVFR(telemetry);
				}	
					break;
				case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
					mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
					fprintf(stdout, "RC CHANNELS RECEIVED\n");
					setTelemetryRCRaw(telemetry);
				}	
					break;
				default:
					break;
			}

		}
	}		
}


/* Initialise mavlink serial port for telemetry input */
int initMAVLink() {
	
	mavlink_fd = open(TELEMETRY_PORT, O_RDONLY | O_NOCTTY );
	if (mavlink_fd == -1) {
		fprintf(stderr, "Error: Cannot open mavlink telemetry port.\n");
		return -1;
	}

	struct termios2 tio;
	if (ioctl(mavlink_fd, TCGETS2, &tio) == -1) {
		fprintf(stderr, "Error getting serial port settings for MAVLINK telemetry.\n");
		close(mavlink_fd);
		return -1;
	}

	tio.c_cflag &= ~CBAUD;
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;
	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSTOPB;
  	tio.c_cflag &= ~CRTSCTS;

    tio.c_ispeed = MAVLINK_BAUD_RATE;

	// tio.c_cc[VTIME] = 0;
	// tio.c_cc[VMIN] = 0;

	if (ioctl(mavlink_fd, TCSETS2, &tio) == -1) {
		fprintf(stderr, "Error setting serial port settings for MAVLINK telemetry.\n");
		close(mavlink_fd);
		return -1;
	}

	return 1; /* Success */
}

void closeMAVLink() {
	/* Free buffers if needed */
	close(mavlink_fd);
}