#ifndef TRANSMIT_H
#define TRANSMIT_H

/*  Server url for post request  */
#define SERVER_ADDRESS "http://localhost:8000/telemetry"

#define DELAY_MS 500 // half a second


void transmitToServer();

#endif