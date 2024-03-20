#include <stdint.h>
#include <stdio.h>
#include "./mavlink.h"


static int mavlink_fd; // mavlink port file descriptor;

int openMAVLink() {

	mavlink_file = open(MAVLINK_TTY_FILE, O_RDONLY);
	if (mavlink_file == -1) {
		fprintf(stderr, "Error: Cannot open mavlink port.\n");
		return -1;
	}

	// Configure tty port


	return 1; // success
}


int readMAVLink(); // read mavlink packets logic


int freeMAVLink() {
	// free buffers logix
	close(mavlink_fd);
}
