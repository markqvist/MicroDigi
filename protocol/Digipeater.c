#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "device.h"
#include "Digipeater.h"

#define countof(a) sizeof(a)/sizeof(a[0])
#define DECODE_CALL(buf, addr) for (unsigned i = 0; i < sizeof((addr)); i++) { char c = (*(buf)++ >> 1); (addr)[i] = (c == ' ') ? '\x0' : c; }

uint8_t packetBuffer[AX25_MAX_FRAME_LEN]; // Buffer for holding incoming serial data
AX25Ctx *ax25ctx;
Afsk *channel;
Serial *serial;
size_t frame_len;

unsigned long slotTime = 200;
uint8_t p = 255;

void digipeater_processPackets(void) {
    if (frame_len != 0) {
        // We have a packet for digipeating,
        // let's process it
        uint8_t *buf = packetBuffer;

        bool repeat = false;

        DECODE_CALL(buf, dst.call);
        dst.ssid = (*buf++ >> 1) & 0x0F;

        DECODE_CALL(buf, src.call);
        src.ssid = (*buf >> 1) & 0x0F;

        uint8_t rpt_count;
        uint8_t rpt_count_out = 0;
        rpt_hbits = 0x00;
        for (rpt_count = 0; !(*buf++ & 0x01) && (rpt_count < countof(rpt_list)); rpt_count++) {
            DECODE_CALL(buf, rpt_list[rpt_count].call);
            rpt_list[rpt_count].ssid = (*buf >> 1) & 0x0F;

            bool hbit = (*buf >> 7);
            rpt_hbits |= (hbit << rpt_count);

            if (!hbit) {
                // If the H-bit is not set, this path
                // component is active, and we should
                // check whether to digipeat
                AX25Call *path_call = &rpt_list[rpt_count];

                // TODO: add own-call specific relay checking
                if (rpt_list[rpt_count].ssid > 0) {
                    if (memcmp("WIDE", path_call->call, 4) == 0) {
                        repeat = true;
                        uint8_t rssid = rpt_list[rpt_count].ssid - 1;
                        if (rssid == 0) {
                            // If n has reached 0, replace the
                            // WIDE with our own call, and set
                            // the H-bit
                            //memset(rpt_list[rpt_count].call, 0, 6);
                            memcpy(rpt_list_out[rpt_count_out].call, DIGIPEATER_CALLSIGN, 6);
                            rpt_list_out[rpt_count_out].ssid = DIGIPEATER_SSID;
                            rpt_count_out++;
                        } else {
                            // If not, insert our own callsign,
                            // set the H-bit and then add the
                            // Rest of the WIDE, decrementing
                            // the n part
                            memcpy(rpt_list_out[rpt_count_out].call, DIGIPEATER_CALLSIGN, 6);
                            rpt_list_out[rpt_count_out].ssid = DIGIPEATER_SSID;
                            rpt_hbits_out |= (0x01 << rpt_count_out);
                            rpt_count_out++;

                            memcpy(rpt_list_out[rpt_count_out].call, rpt_list[rpt_count].call, 6);
                            rpt_list_out[rpt_count_out].ssid = rssid;
                            rpt_hbits_out &= 0xFF ^ (0x01 << rpt_count_out);
                            rpt_count_out++;
                        }
                    }
                }
            } else {
                memcpy(rpt_list_out[rpt_count_out].call, rpt_list[rpt_count].call, 6);
                rpt_list_out[rpt_count_out].ssid = rpt_list[rpt_count].ssid;
                rpt_hbits_out |= (0x01 << rpt_count_out);
                rpt_count_out++;
            }

        }

        #if SERIAL_DEBUG
            printf_P(PSTR("SRC[%.6s-%d] "), src.call, src.ssid);
            printf_P(PSTR("DST[%.6s-%d] "), dst.call, dst.ssid);
            printf("\nRXd Path (%d): ", rpt_count);
            for (uint8_t i = 0; i < rpt_count; i++) {
                if ((rpt_hbits >> i) & 0x01) {
                    // This path component has beeen
                    // repeated (used).
                    printf_P(PSTR("[%.6s-%d*] "), rpt_list[i].call, rpt_list[i].ssid);
                } else {
                    // Not yet repeated
                    printf_P(PSTR("[%.6s-%d] "), rpt_list[i].call, rpt_list[i].ssid);
                }
            }

            if (repeat) {
                printf("\nTXd Path (%d): ", rpt_count_out);
                for (uint8_t i = 0; i < rpt_count_out; i++) {
                    if ((rpt_hbits_out >> i) & 0x01) {
                        // This path component has beeen
                        // repeated (used).
                        printf_P(PSTR("[%.6s-%d*] "), rpt_list_out[i].call, rpt_list_out[i].ssid);
                    } else {
                        // Not yet repeated
                        printf_P(PSTR("[%.6s-%d] "), rpt_list_out[i].call, rpt_list_out[i].ssid);
                    }
                }    
            }
            
            puts("\n");
        #endif


        // Send it out!
        //if (repeat) digipeater_csma(ax25ctx, packetBuffer, frame_len);

        // Reset frame_len to 0
        frame_len = 0;
    }
}

void digipeater_init(AX25Ctx *ax25, Afsk *afsk, Serial *ser) {
    ax25ctx = ax25;
    serial = ser;
    channel = afsk;
    frame_len = 0;
}

void digipeater_csma(AX25Ctx *ctx, uint8_t *buf, size_t len) {
    bool sent = false;
    while (!sent) {
        if(!channel->hdlc.receiving) {
            uint8_t tp = rand() & 0xFF;
            if (tp < p) {
                ax25_sendRaw(ctx, buf, len);
                sent = true;
            } else {
                ticks_t start = timer_clock();
                long slot_ticks = ms_to_ticks(slotTime);
                while (timer_clock() - start < slot_ticks) {
                    cpu_relax();
                }
            }
        } else {
            while (!sent && channel->hdlc.receiving) {
                // Continously poll the modem for data
                // while waiting, so we don't overrun
                // receive buffers
                ax25_poll(ax25ctx);

                if (channel->status != 0) {
                    // If an overflow or other error
                    // occurs, we'll back off and drop
                    // this packet silently.
                    channel->status = 0;
                    sent = true;
                }
            }
        }

    }
}

void digipeater_messageCallback(AX25Ctx *ctx) {
    if (frame_len == 0) {
        cli();

        for (size_t i = 0; i < ctx->frame_len; i++) {
            packetBuffer[i] = ctx->buf[i];
        }

        frame_len = ctx->frame_len;
        sei();
    }
}
