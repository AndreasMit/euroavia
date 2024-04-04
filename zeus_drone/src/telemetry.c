#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdatomic.h>
#include "./telemetry.h"


#ifdef MAVLINK_TELEMETRY
#include "./mavlink_layer.h"
#endif


/*  Initialised Telemetry   */
int initTelemetry(telemetry_info_t *telemetry) {
    int8_t ret;

    telemetry = (telemetry_info_t *)malloc(sizeof(telemetry_info_t));
    if (telemetry == NULL) {
        fprintf(stderr, "Error allocating memory for telemetry.\n");
        return -1;
    }

    telemetry->attitude = NULL;
    telemetry->global_pos = NULL;
    telemetry->gps = NULL;
    telemetry->rc_data = NULL;
    telemetry->status = NULL;
    telemetry->vfr = NULL;
    atomic_flag_clear(&telemetry->lock);


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
    handleTelemetry(telemetry);
#endif

}


/*  Terminates Telemetry    */
void freeTelemetry(telemetry_info_t *telemetry) {

#ifdef MAVLINK_TELEMETRY
    closeMAVLink();
#endif

    if (telemetry != NULL)
        free(telemetry);

}


