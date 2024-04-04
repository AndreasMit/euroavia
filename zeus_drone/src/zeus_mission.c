#define _XOPEN_SOURCE 700

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include "s_bus.h"
#include "telemetry.h"
#include "transmit.h"


telemetry_info_t *telemetry = NULL;


void sigint_handler(int signum) {
	fprintf(stdout, "Received SIGINT. Cleaning up and exiting.\n");
	freeTelemetry(telemetry);
	exit(signum);
}

int main() {

    // Set up signal handler for SIGINT
    struct sigaction sigint_action;
    sigint_action.sa_handler = sigint_handler;
    sigemptyset(&sigint_action.sa_mask);
    sigint_action.sa_flags = 0;
    sigaction(SIGINT, &sigint_action, NULL);

	// Initialise telemetry
	if (initTelemetry(telemetry) == -1) {
		fprintf(stderr, "Error: Cannot initialise telemetry. Terminating program...\n");
		return 1;
	}

	// int fd = sbus_open();

	// struct SBUSFrame msg;
	// initialize_sbus_frame(&msg);

	while (1) {
		// set_sbus_channel(&msg, CH_1, 990);
		// set_sbus_channel(&msg, CH_2, 992);
		// set_sbus_channel(&msg, CH_3, 1299);
		// set_sbus_channel(&msg, CH_4, 989);
		// set_sbus_channel(&msg, CH_5, 172);
		// set_sbus_channel(&msg, CH_6, 172);
		// set_sbus_channel(&msg, CH_7, 172);
		// set_sbus_channel(&msg, CH_8, 172);
		// set_sbus_channel(&msg, CH_9, 172);
		// set_sbus_channel(&msg, CH_10, 172);
		// set_sbus_channel(&msg, CH_11, 1031);
		// set_sbus_channel(&msg, CH_12, 1810);
		// set_sbus_channel(&msg, CH_13, 992);
		// set_sbus_channel(&msg, CH_14, 992);
		// set_sbus_channel(&msg, CH_15, 992);
		// set_sbus_channel(&msg, CH_16, 992);
		// sbus_write(fd, &msg);
		processTelemetry(telemetry);
	}

	return 0;
}
