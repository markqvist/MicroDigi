#ifndef _PROTOCOL_DIGIPEATER
#define _PROTOCOL_DIGIPEATER 0x03

#include "../hardware/AFSK.h"
#include "../hardware/Serial.h"
#include "../util/time.h"
#include "config.h"
#include "AX25.h"

typedef struct AX25Call {
    char call[6];
    uint8_t ssid;
} AX25Call;

#define AX25_CALL(str, id) {.call = (str), .ssid = (id) }
#define AX25_MAX_RPT 8

uint8_t  rpt_hbits;
uint8_t  rpt_hbits_out;
AX25Call rpt_list[AX25_MAX_RPT];
AX25Call rpt_list_out[AX25_MAX_RPT];
AX25Call src;
AX25Call dst;

void digipeater_init(AX25Ctx *ax25, Afsk *afsk, Serial *ser);
void digipeater_csma(AX25Ctx *ctx, uint8_t *buf, size_t len);
void digipeater_messageCallback(AX25Ctx *ctx);
bool is_duplicate(uint8_t crcl, uint8_t crch);
void digipeater_processPackets(void);

#endif