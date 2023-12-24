#include "stdint.h"
#define SBUS_TTY_FILE  "/dev/ttySOMETHING"
#define SBUS_FRAME_LENGTH 25
#define SBUS_NUM_CHANNELS 16
#define SBUS_BAUD_RATE 100000
// 200000 Baud Rate for "Fast SBUS"


// Success / Error codes
#define SBUS_SUCCESS 5
#define SBUS_ERROR -1


// 16 bit channels. Only 11 bits will be used per channel!

struct SBUSFrame {
    uint8_t startByte;
    uint16_t channels[SBUS_NUM_CHANNELS];
    uint8_t flagByte;
    uint8_t endByte;
};


uint8_t sbus_write(const int sbusFile, const struct SBUSFrame *msg);

int sbus_open();

void clear_msg_channels(struct SBUSFrame *msg);

uint8_t initialize_sbus_msg(struct SBUSFrame *msg);
