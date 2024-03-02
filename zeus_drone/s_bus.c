#include "./s_bus.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "fcntl.h"
#include <sys/types.h>
#include "unistd.h"
#include "asm/termbits.h"
#include "sys/ioctl.h"

uint8_t sbus_write(const int sbusFile, const struct SBUSFrame *msg) {


    if (!sbusFile || !msg) {
        perror("Error: Can't write to specific tty file or write specific message");
        close(sbusFile);
        return SBUS_ERROR;
    }

    uint8_t buf[SBUS_FRAME_LENGTH];
    memset(buf, 0, sizeof(buf));

    // Header Frame
    buf[0] = msg->startByte;

    // Channels
    buf[1] = (uint8_t)((msg->channels[CH_1] & 0x07FF));
    buf[2] = (uint8_t)((msg->channels[CH_1] & 0x07FF) >> 8 |
                        (msg->channels[CH_2] & 0x07FF) << 3);
    buf[3] = (uint8_t)((msg->channels[CH_2] & 0x07FF) >> 5 |
                        (msg->channels[CH_3] & 0x07FF) << 6);
    buf[4] = (uint8_t)((msg->channels[CH_3] & 0x07FF) >> 2);
    buf[5] = (uint8_t)((msg->channels[CH_3] & 0x07FF) >> 10 |
                        (msg->channels[CH_4] & 0x07FF) << 1);
    buf[6] = (uint8_t)((msg->channels[CH_4] & 0x07FF) >> 7 |
                        (msg->channels[CH_5] & 0x07FF) << 4);
    buf[7] = (uint8_t)((msg->channels[CH_5] & 0x07FF) >> 4 |
                        (msg->channels[CH_6] & 0x07FF) << 7);
    buf[8] = (uint8_t)((msg->channels[CH_6] & 0x07FF) >> 1);
    buf[9] = (uint8_t)((msg->channels[CH_6] & 0x07FF) >> 9 |
                        (msg->channels[CH_7] & 0x07FF) << 2);
    buf[10] = (uint8_t)((msg->channels[CH_7] & 0x07FF) >> 6 |
                        (msg->channels[CH_8] & 0x07FF) << 5);
    buf[11] = (uint8_t)((msg->channels[CH_8] & 0x07FF) >> 3);
    buf[12] = (uint8_t)((msg->channels[CH_9] & 0x07FF));
    buf[13] = (uint8_t)((msg->channels[CH_9] & 0x07FF) >> 8 |
                        (msg->channels[CH_10] & 0x07FF) << 3);
    buf[14] = (uint8_t)((msg->channels[CH_10] & 0x07FF) >> 5 |
                        (msg->channels[CH_11] & 0x07FF) << 6);
    buf[15] = (uint8_t)((msg->channels[CH_11] & 0x07FF) >> 2);
    buf[16] = (uint8_t)((msg->channels[CH_11] & 0x07FF) >> 10 |
                        (msg->channels[CH_12] & 0x07FF) << 1);
    buf[17] = (uint8_t)((msg->channels[CH_12] & 0x07FF) >> 7 |
                        (msg->channels[CH_13] & 0x07FF) << 4);
    buf[18] = (uint8_t)((msg->channels[CH_13] & 0x07FF) >> 4 |
                        (msg->channels[CH_14] & 0x07FF) << 7);
    buf[19] = (uint8_t)((msg->channels[CH_14] & 0x07FF) >> 1);
    buf[20] = (uint8_t)((msg->channels[CH_14] & 0x07FF) >> 9 |
                        (msg->channels[CH_15] & 0x07FF) << 2);
    buf[21] = (uint8_t)((msg->channels[CH_15] & 0x07FF) >> 6 |
                        (msg->channels[CH_16] & 0x07FF) << 5);
    buf[22] = (uint8_t)((msg->channels[CH_16] & 0x07FF) >> 3);

    // Flag Byte
    buf[23] = msg->flagByte;

    // End Frame
    buf[SBUS_FRAME_LENGTH - 1] = msg->endByte;

#ifdef SBUS_INVERTED
    for (uint8_t i = 0; i < SBUS_FRAME_LENGTH; ++i)
	    buf[i] = ~buf[i];
#endif

    ssize_t bytes_written = write(sbusFile, buf, sizeof(buf));
    
    if (bytes_written == -1) {
        perror("Error: Can't write buffer to TTY device");
        close(sbusFile);
        return SBUS_ERROR;
    }

    return SBUS_SUCCESS; // 0 means 
}


int sbus_open() {

    int sbusFile = open(SBUS_TTY_FILE, O_WRONLY);
    if (sbusFile == -1) {
        perror("Error connecting to SBUS.");
	    return SBUS_ERROR;
    }

#ifdef TTY_DEV
    struct termios2 tty;
    // memset(&tty, 0, sizeof(tty));

    if (ioctl(sbusFile, TCGETS2 ,&tty) < 0) {
        perror("Error: Can't get TTY File attributes");
        close(sbusFile);
        return SBUS_ERROR;
    }

    // configure tty settings

    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~(PARODD | CMSPAR);
    tty.c_cflag |= CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CLOCAL;
    tty.c_cflag |= CS8;        // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;  // disable hardware flow control
    tty.c_lflag &= ~ECHO;    // do not echo
    tty.c_lflag &= ~ISIG;    // do not generate signals
    tty.c_oflag &= ~OFILL;  // no fill characters

    // Baud rate
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;  // see CBAUDEX in case this doesnt work!!
    tty.c_ospeed = SBUS_BAUD_RATE;
    tty.c_ispeed = SBUS_BAUD_RATE; // perhaps not needed
    if (ioctl(sbusFile, TCSETS2, &tty) < 0) {
    	perror("Error setting baud rate.");
	return SBUS_ERROR;
    }
#endif
    return sbusFile;
}

void set_sbus_channel(struct SBUSFrame *msg, uint8_t CHANNEL_NO, int value) {
    msg->channels[CHANNEL_NO] = value;
}

void clear_sbus_channels(struct SBUSFrame *msg) {
    for(uint8_t i = 0; i < SBUS_NUM_CHANNELS; ++i) {
        msg->channels[i] = MIN_VALUE;
    }
}

uint8_t initialize_sbus_frame(struct SBUSFrame *msg) {

    msg->startByte = 0x0f;

    clear_sbus_channels(msg);

    // make flagByte 0x00
    msg->flagByte = 0x00;

    msg->endByte = 0x00;

    return SBUS_SUCCESS;
}
