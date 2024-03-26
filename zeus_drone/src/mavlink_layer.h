#ifndef MAVLINK_TELEMETRY_H
#define MAVLINK_TELEMETRY_H

#define MAVLINK_BAUD_RATE 57600


int initMAVLink();
void handleTelemetry();
void closeMAVLink();


#endif
