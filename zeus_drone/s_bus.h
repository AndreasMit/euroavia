#define SBUS_TTY_FILE  "/dev/ttySOMETHING"
#define SBUS_FRAME_LENGTH 25
#define SBUS_NUM_CHANNELS 16
#define SBUS_BAUD_RATE B100000

// Success / Error types
#define SBUS_SUCCESS = 5
#define SBUS_ERROR = -1;


// 16 bit channels. Only 11 bits will be used per channel!

struct SBUSFrame {
    uint8_t startByte;
    uint16_t channels[SBUS_NUM_CHANNELS];
    uint8_t flag;
    uint8_t endByte;
};