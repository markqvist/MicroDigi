#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "device.h"
#include "Digipeater.h"
#include "util/CRC-CCIT.h"

#define countof(a) sizeof(a)/sizeof(a[0])
#define DECODE_CALL(buf, addr) for (unsigned i = 0; i < sizeof((addr)); i++) { char c = (*(buf)++ >> 1); (addr)[i] = (c == ' ') ? '\x0' : c; }

uint8_t packetBuffer[AX25_MAX_FRAME_LEN]; // Buffer for holding incoming serial data
uint8_t packetBufferOut[AX25_MAX_FRAME_LEN];
AX25Ctx *ax25ctx;
Afsk *channel;
Serial *serial;
size_t frame_len;
size_t frame_len_out;
bool csma_waiting = false;

unsigned long slotTime = 200;
uint8_t p = 255;

typedef struct dupl_entry {
    bool active;
    uint8_t crcl;
    uint8_t crch;
    ticks_t timestamp;
} dupl_entry;

#define DUPL_LIST_SIZE 32
#define DUPL_STALE_TIME 30
dupl_entry dupl_list[DUPL_LIST_SIZE];
uint8_t dupl_i = 0;

#if DIGIPEATER_ROLE == ROLE_FILLIN
    static int clamp_n = 1;
#else
    static int clamp_n = DIGIPEATER_CLAMP_N;
#endif

void digipeater_processPackets(void) {
    // If we're waiting in CSMA, drop this packet
    if (csma_waiting) frame_len = 0;

    if (frame_len != 0) {
        // We have a packet for digipeating,
        // let's process it
        uint8_t *buf = packetBuffer;
        uint8_t *bufStart = packetBuffer;

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
                        char *p = path_call->call + 4;
                        int n = atoi(p);
                        int N = rpt_list[rpt_count].ssid;
                        bool dupl_match = false;

                        if (n <= clamp_n && N <= clamp_n && !dupl_match) {
                            repeat = true;
                            frame_len_out = 0;
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

        if (repeat) {
            // Calculate payload length
            int payloadLength = frame_len - (buf - bufStart);
            //printf_P(PSTR("Payload length: %d"), payloadLength);

            // Init outgoing buffer to all zeroes
            memset(packetBufferOut, 0, AX25_MAX_FRAME_LEN);

            // We need to calculate a CRC checksum for
            // the src, dst and information fields only,
            // used to check for duplicates.
            ax25ctx->crc_out = CRC_CCIT_INIT_VAL;

            char c;

            // Add destination address
            for (unsigned i = 0; i < sizeof(dst.call); i++) {
                c = dst.call[i];
                if (c == '\x0') c = ' ';
                c = c << 1;
                packetBufferOut[frame_len_out++] = c;
                // Update CRC
                ax25ctx->crc_out = update_crc_ccit(c, ax25ctx->crc_out);
            }
            packetBufferOut[frame_len_out++] = 0x60 | (src.ssid << 1);

            // Add source address
            for (unsigned i = 0; i < sizeof(src.call); i++) {
                c = src.call[i];
                if (c == '\x0') c = ' ';
                c = c << 1;
                packetBufferOut[frame_len_out++] = c;
                // Update CRC
                ax25ctx->crc_out = update_crc_ccit(c, ax25ctx->crc_out);
            }
            packetBufferOut[frame_len_out++] = 0x60 | (src.ssid << 1);

            // Add path
            for (int i = 0; i < rpt_count_out; i++) {
                AX25Call p = rpt_list_out[i];
                bool set_hbit = false;
                if ((rpt_hbits_out >> i) & 0x01) set_hbit = true;
                for (unsigned i = 0; i < sizeof(p.call); i++) {
                    c = p.call[i];
                    if (c == '\x0') c = ' ';
                    c = c << 1;
                    packetBufferOut[frame_len_out++] = c;
                }
                packetBufferOut[frame_len_out++] = 0x60 | (p.ssid << 1) | (set_hbit ? 0x80 : 0x00) | (i == rpt_count_out-1 ? 0x01 : 0);
            }

            packetBufferOut[frame_len_out++] = AX25_CTRL_UI;
            packetBufferOut[frame_len_out++] = AX25_PID_NOLAYER3;

            // Add payload
            for (int i = 0; i < payloadLength-2; i++) {
                if (i > 1) {
                    packetBufferOut[frame_len_out++] = buf[i];
                    // Update CRC
                    ax25ctx->crc_out = update_crc_ccit(buf[i], ax25ctx->crc_out);
                }
            }
            uint8_t crcl = (ax25ctx->crc_out & 0xff) ^ 0xff;
            uint8_t crch = (ax25ctx->crc_out >> 8) ^ 0xff;

            if (!is_duplicate(crcl, crch)) {
                // Send it out!
                digipeater_csma(ax25ctx, packetBufferOut, frame_len_out);

                // Add packet to duplicate checklist
                dupl_list[dupl_i].crcl = crcl;
                dupl_list[dupl_i].crch = crch;
                dupl_list[dupl_i].timestamp = timer_clock();
                dupl_list[dupl_i].active = true;
                dupl_i = (dupl_i + 1) % DUPL_LIST_SIZE;
            }
        }

        // Reset frame_len to 0
        frame_len = 0;
    }
}

bool is_duplicate(uint8_t crcl, uint8_t crch) {
    ticks_t now = timer_clock();
    for (int i = 0; i < DUPL_LIST_SIZE; i++) {
        if (dupl_list[i].active && now - dupl_list[i].timestamp < s_to_ticks(DUPL_STALE_TIME)) {
            if (dupl_list[i].crcl == crcl && dupl_list[i].crch == crch) {
                return true;
            }
        } else {
            dupl_list[i].active = false;
        }
    }
    return false;
}

void digipeater_init(AX25Ctx *ax25, Afsk *afsk, Serial *ser) {
    ax25ctx = ax25;
    serial = ser;
    channel = afsk;
    frame_len = 0;
}

void digipeater_csma(AX25Ctx *ctx, uint8_t *buf, size_t len) {
    bool sent = false;
    csma_waiting = true;
    while (!sent) {
        if(!channel->hdlc.receiving) {
            uint8_t tp = rand() & 0xFF;
            if (tp < p) {
                ax25_sendRaw(ctx, buf, len);
                sent = true;
                csma_waiting = false;
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
                    csma_waiting = false;
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
