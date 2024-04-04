#ifndef MAVLINK_TELEMETRY_H
#define MAVLINK1_TELEMETRY_H

#include "telemetry.h"

#define MAVLINK_BAUD_RATE 57600


int initMAVLink();
void handleTelemetry(telemetry_info_t *telemetry);
void closeMAVLink();


#endif
