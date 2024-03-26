#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "./telemetry.h"


#ifdef MAVLINK_TELEMETRY
#include "./mavlink_layer.h"
#endif


/*  Initialised Telemetry   */
int initTelemetry() {
    int8_t ret;

#ifdef MAVLINK_TELEMETRY
    ret = initMAVLink();
#endif

if (ret == -1) 
    fprintf(stderr, "Error initialising telemetry.\n");
else
    fprintf(stdout, "Telemetry Initialised.\n");

return ret;
}


/*  Gets telemetry data  */
void processTelemetry(telemetry_info_t *telemetry) {
#ifdef MAVLINK_TELEMETRY
    handleTelemetry();
#endif
}


/*  Terminates Telemetry    */
void freeTelemetry() {

#ifdef MAVLINK_TELEMETRY
    closeMAVLink();
#endif

}


