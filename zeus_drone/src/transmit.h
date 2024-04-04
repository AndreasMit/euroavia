#ifndef TRANSMIT_H
#define TRANSMIT_H

/*  Server url for post request  */
#define SERVER_ADDRESS_LOGS "http://localhost/api/telemetry"
#define SERVER_ADDRESS_MISSION_CONFIGURATION "http://localhost/api/mission"

/*  Status to know what the message sent is */
#define MISSION_CONF_STATUS 2
#define MISSION_LOG_STATUS  3


#define DELAY_MS 500 // half a second

#define MISSION_X_MISSIONID_LEN 7

/*  If initMissionTransmission fails due to telemetry not being available yet this status code is used.    */
#define WAITING_TELEMETRY   -2


int initMissionTransmission();
void sendTelemetryLog();

#endif