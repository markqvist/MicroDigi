#ifndef CONFIG_H
#define CONFIG_H

// Configure the digipeaters callsign and SSID
#define DIGIPEATER_CALLSIGN "NOCALL"
#define DIGIPEATER_SSID 0

// Configure digipeater type. Available values
// are ROLE_WIDENN and ROLE_FILLIN
#define DIGIPEATER_ROLE ROLE_WIDENN

// Define the max hop count that the digipeater
// will relay. This looks at both n and N.
#define DIGIPEATER_CLAMP_N 2

// Whether to digipeat packets that specify a
// path directly through the digipeaters call-
// sign and SSID
#define SPECIFIC_DIGIPEAT true

// Set CSMA slot time
#define DIGIPEATER_CSMA_SLOT_TIME 200

// Set CSMA persistence value
#define DIGIPEATER_CSMA_PERSISTENCE 127

// How many packets to keep in the duplicate
// checklist
#define DUPL_LIST_SIZE 32

// How long a packet is kept in the duplicate
// checklist
#define DUPL_STALE_TIME 30

#endif