#ifndef SBUS_H
#define SBUS_H

#include "stdint.h"

#define SBUS_TTY_FILE  "/dev/ttyUSB0"
//#define SBUS_TTY_FILE "./example"
#define TTY_DEV

#define SBUS_FRAME_LENGTH 25
#define SBUS_NUM_CHANNELS 16
#define SBUS_BAUD_RATE 100000

/*	Min/Max Value Of Transmitter Sent	*/
#define MIN_VALUE 172
#define MAX_VALUE 1810

/*	Minimum Time Between Each Packet (ms)	*/
#define SBUS_PACKETS_DELAY_TIME 10

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10
#define CH_12 11
#define CH_13 12
#define CH_14 13
#define CH_15 14
#define CH_16 15

/*	DRONE CHANNELS	*/
#define THROTTLE CH_3
#define ROLL CH_1
#define PITCH CH_2
#define YAW CH_3
/*	DRONE CHANNELS	*/

/*	SBUS Success/Error Codes Begin	*/
#define SBUS_SUCCESS 5
#define SBUS_ERROR -1
#define SBUS_INTERVAL 2
/*	SBUS Success/Error Codes End	*/

struct SBUSFrame {
    uint8_t startByte;
    uint16_t channels[SBUS_NUM_CHANNELS];
    uint8_t flagByte;
    uint8_t endByte;
};

uint8_t sbus_write(const int sbusFile, const struct SBUSFrame *msg);
int sbus_open();
void set_sbus_channel(struct SBUSFrame *msg, uint8_t CHANNEL_NO, int value);
void clear_sbus_channels(struct SBUSFrame *msg);
uint8_t initialize_sbus_frame(struct SBUSFrame *msg);

#endif
