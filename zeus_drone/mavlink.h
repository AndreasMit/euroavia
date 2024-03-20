#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_TTY_FILE "./dev/ttyEXAMPLE"
#define MAVLINK_BAUD_RATE 57600



#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_NUM_CHECKSUM_BYTES 2

#define MAVLINK_MSG_ID 30

typedef struct __mavlink_message {
	uint16_t checksum; 	// sent at the end of the packet
	uint8_t magic;		// protocol magic marker
	uint8_t len;		// length of payload
	uint8_t seq;		// sequence of packet
	uint8_t sysid;		// ID if message system/aircraft
	uint8_t compid;		// ID of the message sender component
	uint8_t msgid;		// ID of message in payload
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES + 7) / 8];
} mavlink_message_to;

typedef struct __mavlink_gps_raw_int_t {
	uint64_t time_usec; // Timestamp (UNIX epoch)
	int32_t lat;
	int32_t lon;
	int32_t alt;
	uint16_t eph;
	uint16_t epv;
	uint16_t vel;
	uint16_t cog;
	uint8_t fix_type;
	uint8_t satellites_visible;
} mavlink_gps_raw_int_t;


int openMAVLink();
int readMAVLink();
int freeMAVLink();

#endif
