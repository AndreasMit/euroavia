#include "stdio.h"
#include "s_bus.h"
#include "unistd.h"

int main() {

	int fd = sbus_open();
	// initialize SBUSFrame struct
	
	struct SBUSFrame msg;
	initialize_sbus_frame(&msg);
	set_sbus_channel(&msg, THROTTLE, 1000);
	sbus_write(fd, &msg);		
	sleep(2);
	clear_sbus_channels(&msg);
	sbus_write(fd, &msg);
	return 0;
}
