#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/termios.h>
#include <stdbool.h>
#include <./telemetry.h>
#include <./mavlink_layer.h>
#include <mavlink/common/mavlink.h>


static int mavlink_fd;

mavlink_status_t status;
mavlink_message_t msg;

const int chan = MAVLINK_COMM_0;

mavlink_global_position_int_t global_position;
mavlink_gps_raw_int_t gps_raw;
mavlink_battery_status_t battery_status;


/*	Parse 1 byte at a time	*/
int handleTelemetry(telemetry_info_t *telemetry) {
	/* Read 1 byte from serial port */
	uint8_t byte;
	while (read(mavlink_fd, byte, sizeof(byte))) {

		if (mavlink_parse_char(chan, byte, &msg, &status)) {
			fprintf(stdout, "Received message with ID %d, sequence: %d from compontent %d of system %d\n", msg.msgid, msg.seq, msg.sysid);
			/* Paylod decoding */
			switch(msg.msgid) {
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
					mavlink_msg_global_position_int_decode(&msg, &global_position);
				}
					break;
				case MAVLINK_MSG_ID_GPS_STATUS: {
					mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
				}
					break;
				case MAVLINK_MSG_ID_BATTERY_STATUS: {
					mavlink_msg_battery_status_decode(&msg, &battery_status);
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
	
	mavlink_fd = open(TELEMETRY_PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK );
	if (mavlink_fd == -1) {
		fprintf(stderr, "Error: Cannot open mavlink telemetry port.\n");
		return -1;
	}

	struct termios2 tio;
	if (ioctl(mavlink_fd, TCGETS2, &tio) == -1) {
		fprintf(stderr, "Error getting serial port settings for MAVLINK telemetry.\n");
		vlose(mavlink_fd);
		return -1;
	}

	tio.c_cflag &= ~CBAUD;
	
	tio.c_cflag &= ~CSIZE;
	tio.c_cflag |= CS8;
	tio.c_cflag &= ~PARENB;
	tio.c_cflag &= ~CSTOPB;

    tio.c_ispeed = MAVLINK_BAUD_RATE;

	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN] = 0;

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