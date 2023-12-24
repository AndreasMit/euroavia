#include "./s_bus.h";
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "fcntl.h"
#include "unistd.h"
#include "termios.h"




int sbus_write(int sbusFile, const struct SBUSFrame *msg) {


    if (!sbusFile || !msg) {
        perror("Error: Can't write to specific tty file or write specific message");
        close(sbusFile);
        return SBUS_ERROR;
    }

    uint8_t buf[SBUS_FRAME_LENGTH];
    memset(buf, 0, sizeof(buf));
    // Header Frame
    buf[0] = msg->startByte;
    // End Frame
    buf[SBUS_FRAME_LENGTH - 1] = msg->endByte;

    uint8_t buffer_counter = 1;
    for( uint8_t channel_counter = 1; channel_counter < SBUS_NUM_CHANNELS; ++channel_counter ) {
        if (channel_counter == 1) {
            buf[buffer_counter++] = msg->channels[channel_counter] & 0xFF;
            continue;
        }
        buf[buffer_counter++] = ((msg->channels[channel_counter - 1] >> 8) & 0b111) |  ((msg->channels[channel_counter] & 0b11111) << 3);   
    }

    // DO SOMETHING WITH FLAG byte
    ssize_t bytes_written = write(sbusFile, buf, sizeof(buf));
    
    if (bytes_written == -1) {
        perror("Error: Can't write buffer to TTY device");
        close(sbusFile);
        return SBUS_ERROR;
    }

    return SBUS_SUCCESS; // 0 means 
}


int sbus_open() {

    int sbusFile = open(TTY_FILE_PATH, 0_WRONLY);
    if (sbusFile == -1) {
        perror("Error connecting to SBUS.")
		exit(EXIT_FAILURE);
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(sbusFile, &tty) != 0) {
        perror("Error: Can't get TTY File attributes");
        close(sbusFile);
        return SBUS_ERROR;
    }

    // set output Baud Rate
    cfsetospeed(&tty, SBUS_BAUD_RATE);

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

    return sbusFile;
}


void initialize_sbus_msg(struct SBUSFrame *msg) {

    sbusFrame->startByte = 0x0F;

    memset(msg->channels, 0, sizeof(msg->channels));

    // INIT THE FLAG TO SOMETHING!
    msg->endByte = 0x00;

    return SBUS_SUCCESS;
}