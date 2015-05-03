#include <stdbool.h>
#include <avr/io.h>
#include <string.h>

#include "device.h"
#include "util/FIFO.h"
#include "util/time.h"
#include "hardware/AFSK.h"
#include "hardware/Serial.h"
#include "protocol/AX25.h"
#include "protocol/Digipeater.h"

Serial serial;
Afsk modem;
AX25Ctx AX25;

unsigned long custom_preamble = CONFIG_AFSK_PREAMBLE_LEN;
unsigned long custom_tail = CONFIG_AFSK_TRAILER_LEN;

static void ax25_callback(struct AX25Ctx *ctx) {
    digipeater_messageCallback(ctx);
}

void init(void) {
    sei();

    AFSK_init(&modem);
    ax25_init(&AX25, &modem.fd, ax25_callback);
    digipeater_init(&AX25, &modem, &serial);

    serial_init(&serial);
    stdout = &serial.uart0;
    stdin  = &serial.uart0;

}

int main (void) {
    init();

    while (true) {
        ax25_poll(&AX25);
        digipeater_processPackets();
        
        if (serial_available(0)) {
            //char sbyte = uart0_getchar_nowait();
            // Do something with the data :)
        }
    }

    return(0);
}