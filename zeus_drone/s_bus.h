#define SBUS_TTY_FILE  "/dev/ttySOMETHING"
#define SBUS_FRAME_LENGTH 25
#define SBUS_NUM_CHANNELS 16
#define SBUS_BAUD_RATE B100000
// 200000 Baud Rate for "Fast SBUS"


// Success / Error codes
#define SBUS_SUCCESS 5
#define SBUS_ERROR -1;


// 16 bit channels. Only 11 bits will be used per channel!

struct SBUSFrame {
    uint8_t startByte;
    uint16_t channels[SBUS_NUM_CHANNELS];
    uint8_t flagByte;
    uint8_t endByte;
};