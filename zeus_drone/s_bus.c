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
    buf[1] = (uint8_t)((msg->channels[0] & 0x07FF));
    buf[2] = (uint8_t)((msg->channels[0] & 0x07FF) >> 8 |
                        (msg->channels[1] & 0x07FF) << 3);
    buf[3] = (uint8_t)((msg->channels[1] & 0x07FF) >> 5 |
                        (msg->channels[2] & 0x07FF) << 6);
    buf[4] = (uint8_t)((msg->channels[2] & 0x07FF) >> 2);
    buf[5] = (uint8_t)((msg->channels[2] & 0x07FF) >> 10 |
                        (msg->channels[3] & 0x07FF) << 1);
    buf[6] = (uint8_t)((msg->channels[3] & 0x07FF) >> 7 |
                        (msg->channels[4] & 0x07FF) << 4);
    buf[7] = (uint8_t)((msg->channels[4] & 0x07FF) >> 4 |
                        (msg->channels[5] & 0x07FF) << 7);
    buf[8] = (uint8_t)((msg->channels[5] & 0x07FF) >> 1);
    buf[9] = (uint8_t)((msg->channels[5] & 0x07FF) >> 9 |
                        (msg->channels[6] & 0x07FF) << 2);
    buf[10] = (uint8_t)((msg->channels[6] & 0x07FF) >> 6 |
                        (msg->channels[7] & 0x07FF) << 5);
    buf[11] = (uint8_t)((msg->channels[7] & 0x07FF) >> 3);
    buf[12] = (uint8_t)((msg->channels[8] & 0x07FF));
    buf[13] = (uint8_t)((msg->channels[8] & 0x07FF) >> 8 |
                        (msg->channels[9] & 0x07FF) << 3);
    buf[14] = (uint8_t)((msg->channels[9] & 0x07FF) >> 5 |
                        (msg->channels[10] & 0x07FF) << 6);
    buf[15] = (uint8_t)((msg->channels[10] & 0x07FF) >> 2);
    buf[16] = (uint8_t)((msg->channels[10] & 0x07FF) >> 10 |
                        (msg->channels[11] & 0x07FF) << 1);
    buf[17] = (uint8_t)((msg->channels[11] & 0x07FF) >> 7 |
                        (msg->channels[12] & 0x07FF) << 4);
    buf[18] = (uint8_t)((msg->channels[12] & 0x07FF) >> 4 |
                        (msg->channels[13] & 0x07FF) << 7);
    buf[19] = (uint8_t)((msg->channels[13] & 0x07FF) >> 1);
    buf[20] = (uint8_t)((msg->channels[13] & 0x07FF) >> 9 |
                        (msg->channels[14] & 0x07FF) << 2);
    buf[21] = (uint8_t)((msg->channels[14] & 0x07FF) >> 6 |
                        (msg->channels[15] & 0x07FF) << 5);
    buf[22] = (uint8_t)((msg->channels[15] & 0x07FF) >> 3);

    // Flag Byte
    buf[23] = msg->flagByte;

    // End Frame
    buf[SBUS_FRAME_LENGTH - 1] = msg->endByte;


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
	exit(EXIT_FAILURE);
    }

    struct termios2 tty;
    memset(&tty, 0, sizeof(tty));

    if (ioctl(sbusFile, TCGETS2 ,&tty) < 0) {
        perror("Error: Can't get TTY File attributes");
        close(sbusFile);
        return SBUS_ERROR;
    }

    // configure tty settings
    tty.c_cflag |= PARENB;
    tty.c_cflag &= ~PARODD;
    tty.c_cflag |= CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;        // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;  // disable hardware flow control
    tty.c_lflag &= ~ECHO;    // do not echo
    tty.c_lflag &= ~ISIG;    // do not generate signals
    tty.c_oflag &= ~OFILL;  // no fill characters

    // Baud rate
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ospeed = SBUS_BAUD_RATE;

    if (ioctl(sbusFile, TCSETS2, &tty) < 0) {
    	perror("Error setting baud rate.");
	return SBUS_ERROR;
    }


    return sbusFile;
}


void clear_msg_channels(struct SBUSFrame *msg) {
    memset(msg->channels, 0, sizeof(msg->channels));
}

uint8_t initialize_sbus_msg(struct SBUSFrame *msg) {

    msg->startByte = 0x0F;

    memset(msg->channels, 0, sizeof(msg->channels));

    // make flagByte 0x00
    msg->flagByte = 0x00;

    // INIT THE FLAG TO SOMETHING!
    msg->endByte = 0x00;

    return SBUS_SUCCESS;
}
