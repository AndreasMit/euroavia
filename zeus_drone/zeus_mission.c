#include "stdio.h"
#include "s_bus.h"


int main() {

	// initialize SBUSFrame struct
	struct SBUSFrame msg;
	initialize_sbus_msg(&msg);
	printf("%s\n", "Hello World");
	return 0;
}
