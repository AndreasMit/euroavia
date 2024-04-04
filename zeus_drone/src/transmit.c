#include <stdlib.h>
#include "telemetry.h"
#include "transmit.h"
#include <curl/curl.h>
#include <time.h>

#ifndef SERVER_ADDRESS_LOGS
    #error "No SERVER URL ADDRESS specified for sending telemetry."
#endif

#ifndef SERVER_ADDRESS_MISSION_CONFIGURATION
    #error "No SERVER URL ADDRESS specified for sending mission configuration."
#endif

void sendBufferToServer(int8_t status, const char *json_data) {

    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);

    curl = curl_easy_init();
    if (curl) {

        if (status == MISSION_CONF_STATUS)
            curl_easy_setopt(curl, CURLOPT_URL, SERVER_ADDRESS_MISSION_CONFIGURATION);
        else if (status == MISSION_LOG_STATUS)
            curl_easy_setopt(curl, CURLOPT_URL, SERVER_ADDRESS_LOGS);
        else {
            fprintf(stderr, "Invalid mission packet status.\n");
            curl_easy_cleanup(curl);
            curl_global_cleanup();
        }

        curl_seast_setopt(curl, CURLOPT_POSTFIELDS, json_data);

        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        res = curl_easy_perform(curl);
        if (res != CURLE_OK)
            fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }
    curl_global_cleanup();
}

void generateMissionID(char missionID[]) {
    for (int i = 0; i < MISSION_X_MISSIONID_LEN; i++) {
        missionID[i] = 'A' + rand() % 26; // Capital letters
    }
    missionID[MISSION_X_MISSIONID_LEN] = '\0'; // Null-terminate the string
}

/* Sends mission configuration to server    */
int initMissionTransmission(telemetry_info_t *telemetry) {

    if (telemetry == NULL) {
        fprintf(stdout, "Telemetry is null. Cannot begin transmission to server.\n");
        return WAITING_TELEMETRY;
    }

    if (telemetry->attitude == NULL) {
        fprintf(stdout, "Telemetry attitude is still null. Cannot begin transmission to server.\n");
        return WAITING_TELEMETRY;
    }

    if (telemetry->gps == NULL) {
        fprintf(stdout, "Telemetry gps is still null. Cannot begin transmission to server.\n");
        return WAITING_TELEMETRY;
    }

    char missionID[MISSION_X_MISSIONID_LEN + 1]; // +1 for null terminator
    generateMissionID(missionID);

    char mission_configuration_json[1024];
    snprintf(mission_configuration_json, sizeof(mission_configuration_json),
                       "{"
                       "    \"missionID\": \"%s\","
                       "    \"metricsConfiguration\": {"
                       "        \"roll\": {"
                       "            \"representation\": \"line\","
                       "            \"units\": \"m/s\""
                       "        },"
                       "        \"pitch\": {"
                       "            \"representation\": \"line\","
                       "            \"units\": \"m\""
                       "        },"
                       "        \"yaw\": {"
                       "            \"representation\": \"line\","
                       "            \"units\": \"%%\""
                       "        },"
                       "        \"latitude\": {"
                       "            \"representation\": \"number\","
                       "            \"units\": \"\""
                       "        },"
                       "        \"longitude\": {"
                       "            \"representation\": \"number\","
                       "            \"units\": \"\""
                       "        }"
                       "    }"
                       "}", missionID);

    sendBufferToServer(MISSION_CONF_STATUS, mission_configuration_json);

    return 1; /* Success */
}


/* Makes buffer from telemetry fields and calls sendBufferToServer() */
void sendTelemetryLog(telemetry_info_t *telemetry) {

    char *json_buffer;

    /*  get spinlock/mutex   */
    
    /*  make buffer */

    /* free spinlock/mutex  */
    sendBufferToServer(MISSION_LOG_STATUS, json_buffer);
    
    /*  sleep    */
}


