#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mission.h"

static float _INITIAL_ALTITUDE;

/*  Desired relative altitude for drone to hover (in meters)  */
static const float DESIRED_ALTITUDE = 1.0;

static error_t error;
static error_t previous_error;

static control_t controls;
static gains_t gains;


/*  Initialise gains_t datastructure    */
void setupGains() {
    gains.Kp_yaw = 0;
    gains.Kd_yaw = 0;
    gains.Ki_yaw = 0;

    gains.Kp_pitch = 0.9;
    gains.Kd_pitch = 13;
    gains.Ki_pitch = 0;

    gains.Kp_roll = 0.9;
    gains.Kd_roll = 13;
    gains.Ki_roll = 0;
}

/*  Calibrate drone */
void calibrateDrone() {
    memset(&error, 0, sizeof(error_t));
    memset(&previous_error, 0, sizeof(error_t));
    setupGains();
    fprintf(stdout, "Drone calibrated.\n");
}

/*  Mission code    */
void mission();