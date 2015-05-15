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

#endif