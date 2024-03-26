#include <stdio.h>
#include <unistd.h>
#include "s_bus.h"
#include "telemetry.h"



int main() {

	telemetry_info_t telemetry;
	initTelemetry();
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
		processTelemetry(&telemetry);
	}

	return 0;
}
