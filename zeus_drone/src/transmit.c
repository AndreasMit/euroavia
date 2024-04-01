#include <stdlib.h>
#include "telemetry.h"
#include "transmit.h"
#include <curl/curl.h>
#include <time.h>

#ifndef SERVER_ADDRESS
    #error "No SERVER URL ADDRESS specified for sending telemetry."
#endif



/* Makes buffer from telemetry fields and calls sendBufferToServer() */
void sendToServer(telemetry_info_t *telemetry) {

    char *json_buffer;

    /*  get spinlock/mutex   */
    
    /*  make buffer */

    /* free spinlock/mutex  */
    sendBUfferToServer(json_buffer);
    
    /*  sleep    */
}


void sendBufferToServer(char *json_data) {

    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_ALL);

    curl = curl_easy_init();
    if (curl) {

        curl_easy_setopt(curl, CURLOPT_URL, SERVER_ADDRESS);
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