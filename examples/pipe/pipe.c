
#include "RF24.h"
#include "delay.h"

#include "usb_cdc.h"
#include "tty.h"
#include "nRF24L01.h"
#include "pacer.h"

#include <stddef.h>
#include <string.h>

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

static const usb_cdc_cfg_t usb_cdc_cfg =
{
    .block = 0                  /* Non-blocking I/O  */
};

static spi_cfg_t rf24_spi_cfg = 
{
    .channel = NRF_SPI_CHANNEL,
    .clock_speed_kHz = 1000,
    .cs = NRF_CSN_PIO,
};

static tty_cfg_t usb_cdc_tty_cfg =
{
    .read = (void *)usb_cdc_read,
    .write = (void *)usb_cdc_write
};

static rf24_t rf;

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipe_no = 1;
const uint64_t addr = 0xF0F0F0F0E1LL;

#define MSGLEN 32
static char msgbuf[MSGLEN];

#define POLL_RATE 1000

void command_poll(tty_t *tty) {

    char buffer[32];
    char *str;
    int msg_bytes;

    if (!tty_poll (tty)) {
        return;
    }

    str = tty_gets (tty, buffer, sizeof (buffer));
    if (!str)
        return;

    rf24_openWritingPipe (&rf, addr);
    rf24_stopListening (&rf);

    msg_bytes = min ((unsigned)MSGLEN, strlen(str) + 1);
    strcpy (msgbuf, str);
    msgbuf[msg_bytes - 1] = '\0';

    if (!rf24_write(&rf, msgbuf, MSGLEN)) {
        tty_printf (tty, "write err!\r\n");
    }

    rf24_openReadingPipe (&rf, pipe_no, addr);
    rf24_startListening (&rf);
}

int main(void) {

    usb_cdc_t usb_cdc;
    usb_cdc = usb_cdc_init (&usb_cdc_cfg);
    tty_t *tty = tty_init (&usb_cdc_tty_cfg, usb_cdc);

    rf24_setPowerState (false);
    delay_ms (100);
    rf24_setPowerState (true);
    delay_ms (100);

    rf24_begin (&rf, &rf24_spi_cfg, NRF_CE_PIO, NRF_CSN_PIO);
    rf24_setPayloadSize (&rf, MSGLEN);

    rf24_openReadingPipe (&rf, pipe_no, addr);
    rf24_startListening (&rf);

    pacer_init (POLL_RATE);

    while (true) {

        pacer_wait ();
        command_poll (tty);

        bool done = false;
        if (rf24_available (&rf, NULL)) {
            while (!done) {
                done = rf24_read (&rf, msgbuf, MSGLEN);
                tty_printf (tty, msgbuf);
            }
        }
    }

    return 0;
}

