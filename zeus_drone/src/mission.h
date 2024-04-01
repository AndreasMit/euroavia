#ifndef MISSION_H
#define MISSION_H


/* Axis */
#define X   0 // Roll
#define Y   1 // Pitch
#define Z   2 // Yaw


typedef struct _control {
    int roll;
    int pitch;
    int yaw;
    int throttle;
} control_t;

typedef struct _error {
    float yaw;
    float roll;
    float pitch;
} error_t;

typedef struct __gains {
    float Kp_yaw;
    float Kd_yaw;
    float Ki_yaw;

    float Kp_roll;
    float Kd_roll;
    float Ki_roll;

    float Kp_pitch;
    float Kd_pitch;
    float Ki_pitch;
} gains_t;

#endif