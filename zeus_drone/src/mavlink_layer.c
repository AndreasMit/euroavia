#include <stdio.h>
#include <stdlib.h>
#include <mavlink/common/mavlink.h>


mavlink_status_t status;
mavlink_message_t msg;
int chan = MAVLINK_COMM_0;

mavlink_global_position_int_t global_position;
mavlink_gps_raw_int_t gps_raw;
mavlink_battery_status_t battery_status;


/*	Parse 1 byte at a time	*/
int handleTelemetry() {
	/* Read 1 byte from serial port */
	uint8_t byte; 
	while (byte = read(serial_port, ,1)) {
		if (mavlink_parse_char(chan, byte, &msg, &status)) {
			fprintf(stdout, "REceived message with ID %d, sequence: %d from compontent %d of system %d\n", msg.msgid, msg.seq, msg.sysid);
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
