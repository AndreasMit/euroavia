#include <stdio.h>
#include <stdlib.h>


#ifdef MAVLINK_TELEMETRY
#include "./mavlink_layer.h"
#endif


int initTelemetry();
int processTelemetry(telemetry_info_t *telemetry);
int freeTelemetry();


