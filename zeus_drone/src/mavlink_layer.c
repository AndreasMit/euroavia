#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
// #include <linux/termios.h>
#include "asm/termbits.h"
#include <stdbool.h>
#include "telemetry.h"
#include "mavlink_layer.h"
#include <mavlink/common/mavlink.h>


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
				}
					break;
				case MAVLINK_MSG_ID_GPS_RAW_INT: {
					mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
					fprintf(stdout, "MESSAGE GPS RAW\n");
				}
					break;
				case MAVLINK_MSG_ID_BATTERY_STATUS: {
					mavlink_msg_battery_status_decode(&msg, &battery_status);
					fprintf(stdout, "MESSAGE BATTERY STATUS\n");
				}
					break;
				case MAVLINK_MSG_ID_ATTITUDE: {
					mavlink_msg_attitude_decode(&msg, &attitude);
					fprintf(stdout, "MESSAGE ATTITUDE RECEIVED\n");
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
				}	
					break;
				case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
					mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
					fprintf(stdout, "RC CHANNELS RECEIVED\n");
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