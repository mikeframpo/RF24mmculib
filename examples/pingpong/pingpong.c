
#include "RF24.h"
#include "delay.h"
#include "nRF24L01.h"
#include "pacer.h"

static spi_cfg_t rf24_spi_cfg = 
{
    .channel = NRF_SPI_CHANNEL,
    .clock_speed_kHz = 1000,
    .cs = NRF_CSN_PIO,
};

static rf24_t rf;

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipe_no = 1;
const uint64_t addr = 0xF0F0F0F0E1LL;

// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;

static role_e role;

#define POLL_RATE 1000

void led_flash (int times) {

    pio_config_set (LED_FLASH, PIO_OUTPUT_LOW);

    int i;
    for (i=0; i < times; i++) {
        pio_config_set (LED_FLASH, PIO_OUTPUT_HIGH);
        delay_ms (50);
        pio_config_set (LED_FLASH, PIO_OUTPUT_LOW);
        delay_ms (50);
    }
}

int main(void) {

    role = role_ping_out;
    uint64_t msg = 0xabcdabcdabcdabcd;
    uint64_t response = 0;

    pio_config_set (LED_FLASH, PIO_OUTPUT_LOW);

    rf24_setPowerState (false);
    delay_ms (100);
    rf24_setPowerState (true);
    delay_ms (100);


    rf24_begin (&rf, &rf24_spi_cfg, NRF_CE_PIO, NRF_CSN_PIO);

    rf24_setPayloadSize (&rf, sizeof (msg));
    //rf24_disableCRC (&rf);
    //rf24_disableAutoAck (&rf);
    //rf24_setInterruptSources (&rf, false, false, false);

    if (role == role_ping_out) {
        rf24_openWritingPipe (&rf, addr);
        rf24_stopListening (&rf);
    } else {
        rf24_openReadingPipe (&rf, pipe_no, addr);
        rf24_startListening (&rf);
    }

    pacer_init (POLL_RATE);

    while (true) {

        if (role == role_ping_out) {

            pacer_wait ();

            if (rf24_write(&rf, &msg, sizeof(msg))) {
                led_flash (1);
            } else {
                //flash 5 times for error
                led_flash (5);
            }

            delay_ms (1000);

        } else {
            // pong mode

            if (rf24_available (&rf, 0)) {

                bool done = false;
                while (!done) {

                    done = rf24_read (&rf, &response, sizeof (response));
                }
                led_flash (2);
            }

        }
    }

    return 0;
}

