#include "stdio.h"
#include "s_bus.h"


int main() {

	// initialize SBUSFrame struct
	struct SBUSFrame msg;
	initialize_sbus_frame(&msg);
	printf("%s\n", "Hello World");
	return 0;
}
