/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
//#include "RF24_config.h"
#include "RF24.h"

#include "pio.h"
#include "delay.h"

#define _BV(x) (1 << (x))

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define TPD_TO_STDBY 3

#define WRITE_STATUS_US 100
#define WRITE_TIMEOUT_US 10000
#define WRITE_TIMEOUTS (WRITE_TIMEOUT_US / WRITE_STATUS_US)

/****************************************************************************/

void rf24_ce(rf24_t *rf, bool high)
{
    pio_config_t state = high ? PIO_OUTPUT_HIGH : PIO_OUTPUT_LOW;
    pio_config_set (rf->ce_pin, state);
}

/****************************************************************************/

uint8_t rf24_read_register_buf(rf24_t *rf, uint8_t reg, uint8_t* buf, uint8_t len)
{
    uint8_t status;
    uint8_t blank_tx = 0xff;
    uint8_t reg_address =  R_REGISTER | ( REGISTER_MASK & reg );

    spi_transfer (rf->spi, &reg_address, &status, 1, false);
    while ( len-- )
    {
        bool terminate = len == 0;
        spi_transfer (rf->spi, &blank_tx, buf++, 1, terminate);
    }

    return status;
}

/****************************************************************************/


uint8_t rf24_read_register(rf24_t *rf, uint8_t reg)
{
    uint8_t result;
    uint8_t blank_tx = 0xff;
    uint8_t reg_address = R_REGISTER | ( REGISTER_MASK & reg );
    spi_transfer (rf->spi, &reg_address, 0, 1, false);
    spi_transfer (rf->spi, &blank_tx, &result, 1, true);

    return result;
}


/****************************************************************************/


uint8_t rf24_write_register_buf(rf24_t *rf, uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  uint8_t reg_address = W_REGISTER | ( REGISTER_MASK & reg );
  spi_transfer (rf->spi, &reg_address, &status, 1, false);
  spi_transfer (rf->spi, buf, 0, len, true);

  return status;
}


/****************************************************************************/

uint8_t rf24_write_register(rf24_t *rf, uint8_t reg, uint8_t value)
{
  uint8_t status;

#ifdef NOT_IMPLEMENTED
  IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));
#endif //NOT_IMPLEMENTED

  uint8_t reg_address = W_REGISTER | ( REGISTER_MASK & reg );
  spi_transfer (rf->spi, &reg_address, &status, 1, false);
  spi_transfer (rf->spi, &value, 0, 1, true);

  return status;
}

/****************************************************************************/

uint8_t rf24_write_payload(rf24_t *rf, const void *buf, uint8_t len)
{
    uint8_t status;
    uint8_t reg_address = W_TX_PAYLOAD;

    uint8_t data_len = min (len, rf->payload_size);
    uint8_t blank_len = rf->dynamic_payloads_enabled ? 0 : rf->payload_size - data_len;
    uint8_t blank_tx = 0xff;
    
    //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
    
    spi_transfer (rf->spi, &reg_address, &status, 1, false);
    while ( data_len-- )
    {
        bool terminate = data_len == 0 && blank_len == 0;
        spi_transfer (rf->spi, buf++, 0, 1, terminate);
    }

    while ( blank_len-- )
    {
        bool terminate = blank_len == 0;
        spi_transfer (rf->spi, &blank_tx, 0, 1, terminate);
    }

    return status;
}

/****************************************************************************/

uint8_t rf24_read_payload(rf24_t *rf, void* buf, uint8_t len)
{
    uint8_t status;
    uint8_t data_len = min (len, rf->payload_size);
    uint8_t blank_len = rf->dynamic_payloads_enabled ? 0 : rf->payload_size - data_len;
    uint8_t reg_address = R_RX_PAYLOAD;
    uint8_t blank_tx = 0xff;
    
    //printf("[Reading %u bytes %u blanks]",data_len,blank_len);
    
    spi_transfer (rf->spi, &reg_address, &status, 1, false);
    while ( data_len-- )
    {
        bool terminate = data_len == 0 && blank_len == 0;
        spi_transfer (rf->spi, &blank_tx, buf++, 1, terminate);
    }

    while ( blank_len-- )
    {
        bool terminate = blank_len == 0;
        spi_transfer (rf->spi, &blank_tx, 0, 1, terminate);
    }

    // Clear the status bit

    rf24_write_register (rf, STATUS, _BV(RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(TX_DS) )
    {
        rf24_write_register (rf, STATUS, _BV(TX_DS));
    }

    return status;
}

/****************************************************************************/

uint8_t rf24_flush_rx(rf24_t *rf)
{
    uint8_t status;

    uint8_t reg_address = FLUSH_RX;
    spi_transfer (rf->spi, &reg_address, &status, 1, true);

    return status;
}

/****************************************************************************/

uint8_t rf24_flush_tx(rf24_t *rf)
{
    uint8_t status;

    uint8_t reg_address = FLUSH_TX;
    spi_transfer (rf->spi, &reg_address, &status, 1, true);

    return status;
}

/****************************************************************************/

uint8_t rf24_get_status(rf24_t *rf)
{
    uint8_t status;
    uint8_t reg_address = NOP;

    spi_transfer (rf->spi, &reg_address, &status, 1, true);

    return status;
}

///****************************************************************************/
//
//void RF24::print_status(uint8_t status)
//{
//  printf_P(PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
//           status,
//           (status & _BV(RX_DR))?1:0,
//           (status & _BV(TX_DS))?1:0,
//           (status & _BV(MAX_RT))?1:0,
//           ((status >> RX_P_NO) & B111),
//           (status & _BV(TX_FULL))?1:0
//          );
//}
//
///****************************************************************************/
//
//void RF24::print_observe_tx(uint8_t value)
//{
//  printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
//           value,
//           (value >> PLOS_CNT) & B1111,
//           (value >> ARC_CNT) & B1111
//          );
//}
//
///****************************************************************************/
//
//void RF24::print_byte_register(const char* name, uint8_t reg, uint8_t qty)
//{
//  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
//  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
//  while (qty--)
//    printf_P(PSTR(" 0x%02x"),read_register(reg++));
//  printf_P(PSTR("\r\n"));
//}
//
///****************************************************************************/
//
//void RF24::print_address_register(const char* name, uint8_t reg, uint8_t qty)
//{
//  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
//  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
//
//  while (qty--)
//  {
//    uint8_t buffer[5];
//    read_register(reg++,buffer,sizeof buffer);
//
//    printf_P(PSTR(" 0x"));
//    uint8_t* bufptr = buffer + sizeof buffer;
//    while( --bufptr >= buffer )
//      printf_P(PSTR("%02x"),*bufptr);
//  }
//
//  printf_P(PSTR("\r\n"));
//}
//
///****************************************************************************/
//
//RF24::RF24(uint8_t _cepin, uint8_t _cspin):
//  ce_pin(_cepin), csn_pin(_cspin), wide_band(true), p_variant(false), 
//  payload_size(32), ack_payload_available(false), dynamic_payloads_enabled(false),
//  pipe0_reading_address(0)
//{
//}

/****************************************************************************/

void rf24_setChannel(rf24_t *rf, uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  rf24_write_register(rf, RF_CH, min(channel,max_channel));
}

/****************************************************************************/

void rf24_setPayloadSize(rf24_t *rf, uint8_t size)
{
  const uint8_t max_payload_size = 32;
  rf->payload_size = min(size,max_payload_size);
}

/****************************************************************************/

uint8_t rf24_getPayloadSize(rf24_t *rf)
{
  return rf->payload_size;
}

///****************************************************************************/
//
//static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
//static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
//static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
//static const char * const rf24_datarate_e_str_P[] PROGMEM = {
//  rf24_datarate_e_str_0,
//  rf24_datarate_e_str_1,
//  rf24_datarate_e_str_2,
//};
//static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
//static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
//static const char * const rf24_model_e_str_P[] PROGMEM = {
//  rf24_model_e_str_0,
//  rf24_model_e_str_1,
//};
//static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
//static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
//static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
//static const char * const rf24_crclength_e_str_P[] PROGMEM = {
//  rf24_crclength_e_str_0,
//  rf24_crclength_e_str_1,
//  rf24_crclength_e_str_2,
//};
//static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
//static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
//static const char rf24_pa_dbm_e_str_2[] PROGMEM = "LA_MED";
//static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_HIGH";
//static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = { 
//  rf24_pa_dbm_e_str_0,
//  rf24_pa_dbm_e_str_1,
//  rf24_pa_dbm_e_str_2,
//  rf24_pa_dbm_e_str_3,
//};
//
//void RF24::printDetails(void)
//{
//  print_status(get_status());
//
//  print_address_register(PSTR("RX_ADDR_P0-1"),RX_ADDR_P0,2);
//  print_byte_register(PSTR("RX_ADDR_P2-5"),RX_ADDR_P2,4);
//  print_address_register(PSTR("TX_ADDR"),TX_ADDR);
//
//  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
//  print_byte_register(PSTR("EN_AA"),EN_AA);
//  print_byte_register(PSTR("EN_RXADDR"),EN_RXADDR);
//  print_byte_register(PSTR("RF_CH"),RF_CH);
//  print_byte_register(PSTR("RF_SETUP"),RF_SETUP);
//  print_byte_register(PSTR("CONFIG"),CONFIG);
//  print_byte_register(PSTR("DYNPD/FEATURE"),DYNPD,2);
//
//  printf_P(PSTR("Data Rate\t = %S\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
//  printf_P(PSTR("Model\t\t = %S\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
//  printf_P(PSTR("CRC Length\t = %S\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
//  printf_P(PSTR("PA Power\t = %S\r\n"),pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
//}
//
///****************************************************************************/
//

bool rf24_begin(rf24_t *rf, const spi_cfg_t *spi_cfg, pio_t ce_pin, pio_t csn_pin)
{

    rf->ce_pin = ce_pin;
    rf->csn_pin = csn_pin;
    rf->wide_band = true;
    rf->p_variant = false;
    rf->payload_size = 32;
    rf->pipe0_reading_address = 0;
    rf->ack_payload_available = false;
    rf->ack_payload_length = 0;
    rf->dynamic_payloads_enabled = false;
    rf->autoack_enabled = true;

    // Initialize pins
    pio_config_set (rf->ce_pin, PIO_OUTPUT_LOW);

    // the cfg passed in is const, but we actually want to set a couple
    // of the fields
    spi_cfg_t spi_cpy = *spi_cfg;
    spi_cpy.mode = SPI_MODE_0;
    spi_cpy.bits = 8;

    rf->spi = spi_init (&spi_cpy);
    spi_cs_mode_set (rf->spi, SPI_CS_MODE_FRAME);

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay_ms ( 5 ) ;

    // set the config register back to the default value, for the case that the micro was reset
    // but the radio was not.
    rf24_write_register(rf, CONFIG, _BV(EN_CRC));

    uint8_t config = rf24_read_register (rf, CONFIG);
    if (config != _BV(EN_CRC))
        return false;

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    rf24_write_register (rf, SETUP_RETR,(0x5 << ARD) | (0xf << ARC));

    // Restore our default PA level
    rf24_setPALevel (rf, RF24_PA_MAX ) ;

    // Determine if this is a p or non-p RF24 module and then
    // reset our data rate back to default value. This works
    // because a non-P variant won't allow the data rate to
    // be set to 250Kbps.
    if( rf24_setDataRate (rf, RF24_250KBPS ) )
    {
      rf->p_variant = true ;
    }
    
    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    rf24_setDataRate (rf, RF24_1MBPS ) ;
    
    // Initialize CRC and request 2-byte (16bit) CRC
    rf24_setCRCLength (rf, RF24_CRC_16 ) ;
    
    // Disable dynamic payloads, to match dynamic_payloads_enabled setting
    rf24_write_register(rf, DYNPD,0);
    
    // Reset current status
    // Notice reset and flush is the last thing we do
    rf24_write_register(rf, STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
    
    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    rf24_setChannel(rf, 76);
    
    // Flush buffers
    rf24_flush_rx(rf);
    rf24_flush_tx(rf);

    return true;
}

/****************************************************************************/

void rf24_startListening(rf24_t *rf)
{
    rf24_write_register(rf, CONFIG,
            rf24_read_register(rf, CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
    rf24_write_register(rf, STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    // Restore the pipe0 adddress, if exists
    if (rf->pipe0_reading_address)
      rf24_write_register_buf(rf, RX_ADDR_P0,
              (void *) &rf->pipe0_reading_address, 5);

    // Flush buffers
    rf24_flush_rx(rf);
    rf24_flush_tx(rf);

    // Go!
    rf24_ce(rf, true);

    // wait for the radio to come up
    // the driver originally assumed 130us, however, this only works for the
    // case that an external clock is driving.
    delay_ms (TPD_TO_STDBY);
}

/****************************************************************************/

void rf24_stopListening(rf24_t *rf)
{
    rf24_ce(rf, false);
    rf24_flush_tx(rf);
    rf24_flush_rx(rf);
}

/****************************************************************************/

void rf24_powerDown(rf24_t *rf)
{
  rf24_write_register(rf, CONFIG, rf24_read_register(rf, CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

void rf24_powerUp(rf24_t *rf)
{
  rf24_write_register(rf, CONFIG, rf24_read_register(rf, CONFIG) | _BV(PWR_UP));
}

/******************************************************************/

bool rf24_write(rf24_t *rf, const void* buf, uint8_t len)
{
    bool result = false;

    // Begin the write
    rf24_startWrite(rf, buf, len);

    // ------------
    // At this point we could return from a non-blocking write, and then call
    // the rest after an interrupt

    // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
    // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
    // is flaky and we get neither.

    // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
    // if I tighted up the retry logic.  (Default settings will be 1500us.
    // Monitor the send
    uint8_t status;
    volatile uint32_t timeouts = 0;
    uint8_t observe_tx;

    do
    {
        status = rf24_read_register_buf (rf, OBSERVE_TX, &observe_tx, 1);
#ifdef NOT_IMPLEMENTED
        IF_SERIAL_DEBUG(Serial.print(observe_tx,HEX));
#endif //NOT_IMPLEMENTED

        DELAY_US (WRITE_STATUS_US);
        timeouts++;
    }
    while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( timeouts < WRITE_TIMEOUTS ) );

    // The part above is what you could recreate with your own interrupt handler,
    // and then call this when you got an interrupt
    // ------------

    // Call this when you get an interrupt
    // The status tells us three things
    // * The send was successful (TX_DS)
    // * The send failed, too many retries (MAX_RT)
    // * There is an ack packet waiting (RX_DR)
    bool tx_ok, tx_fail;
    rf24_whatHappened (rf, &tx_ok, &tx_fail, &rf->ack_payload_available);
    
    //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);

    result = tx_ok;
#ifdef NOT_IMPLEMENTED
    IF_SERIAL_DEBUG(Serial.print(result?"...OK.":"...Failed"));
#endif //NOT_IMPLEMENTED

    // Handle the ack packet
    if (rf->ack_payload_available)
    {
        rf->ack_payload_length = rf24_getDynamicPayloadSize(rf);
#ifdef NOT_IMPLEMENTED
        IF_SERIAL_DEBUG(Serial.print("[AckPacket]/"));
        IF_SERIAL_DEBUG(Serial.println(ack_payload_length,DEC));
#endif //NOT_IMPLEMENTED
    }

    // clear the transmit-complete interrupt
    rf24_write_register (rf, STATUS,
    rf24_read_register (rf, STATUS) | _BV(TX_DS) );

    // Yay, we are done.

    // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
    rf24_flush_tx(rf);

    return result;
}

/****************************************************************************/

void rf24_startWrite(rf24_t *rf, const void* buf, uint8_t len)
{
    // Transmitter power-up
    rf24_write_register (rf, CONFIG,
            ( rf24_read_register(rf, CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
    delay_ms (TPD_TO_STDBY);

    // Send the payload
    rf24_write_payload (rf, buf, len);

    // Allons!
    rf24_ce (rf, true);
    DELAY_US (15);
    rf24_ce (rf, false);
}

/****************************************************************************/

uint8_t rf24_getDynamicPayloadSize(rf24_t *rf)
{
#ifdef NOT_IMPLEMENTED
  uint8_t result = 0;

  csn(LOW);
  SPI.transfer( R_RX_PL_WID );
  result = SPI.transfer(0xff);
  csn(HIGH);

  return result;
#endif //NOT_IMPLEMENTED
  return 0;
}

///****************************************************************************/
//
//bool RF24::available(void)
//{
//  return available(NULL);
//}

/****************************************************************************/

bool rf24_available(rf24_t *rf, uint8_t* pipe_num)
{
    uint8_t status = rf24_get_status (rf);

    // Too noisy, enable if you really want lots o data!!
    //IF_SERIAL_DEBUG(print_status(status));

    bool result = ( status & _BV(RX_DR) );

    if (result)
    {
        // If the caller wants the pipe number, include that
        if ( pipe_num )
            *pipe_num = ( status >> RX_P_NO ) & 0x7;
    }

    return result;
}

/****************************************************************************/

bool rf24_read(rf24_t *rf, void* buf, uint8_t len )
{
  // Fetch the payload
  rf24_read_payload(rf, buf, len);

  // was this the last of the data available?
  return rf24_read_register(rf, FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

void rf24_whatHappened(rf24_t *rf, bool *tx_ok, bool *tx_fail, bool *rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = rf24_write_register (rf, STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

  // Report to the user what happened
  *tx_ok = status & _BV(TX_DS);
  *tx_fail = status & _BV(MAX_RT);
  *rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void rf24_openWritingPipe(rf24_t *rf, uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  rf24_write_register_buf(rf, RX_ADDR_P0, (void *) &value, 5);
  rf24_write_register_buf(rf, TX_ADDR, (void *) &value, 5);

  const uint8_t max_payload_size = 32;
  rf24_write_register(rf, RX_PW_P0, min(rf->payload_size, max_payload_size));
}

/****************************************************************************/

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void rf24_openReadingPipe(rf24_t *rf, uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    rf->pipe0_reading_address = address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      rf24_write_register_buf(rf, child_pipe[child], (void *) &address, 5);
    else
      rf24_write_register_buf(rf, child_pipe[child], (void *) &address, 1);

    rf24_write_register(rf, child_payload_size[child], rf->payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    rf24_write_register(rf, EN_RXADDR,
        rf24_read_register(rf, EN_RXADDR) | _BV(child_pipe_enable[child]));
  }
}

///****************************************************************************/
//
//void RF24::toggle_features(void)
//{
//  csn(LOW);
//  SPI.transfer( ACTIVATE );
//  SPI.transfer( 0x73 );
//  csn(HIGH);
//}
//
///****************************************************************************/
//
//void RF24::enableDynamicPayloads(void)
//{
//  // Enable dynamic payload throughout the system
//  write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );
//
//  // If it didn't work, the features are not enabled
//  if ( ! read_register(FEATURE) )
//  {
//    // So enable them and try again
//    toggle_features();
//    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );
//  }
//
//  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
//
//  // Enable dynamic payload on all pipes
//  //
//  // Not sure the use case of only having dynamic payload on certain
//  // pipes, so the library does not support it.
//  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));
//
//  dynamic_payloads_enabled = true;
//}
//
///****************************************************************************/
//
//void RF24::enableAckPayload(void)
//{
//  //
//  // enable ack payload and dynamic payload features
//  //
//
//  write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
//
//  // If it didn't work, the features are not enabled
//  if ( ! read_register(FEATURE) )
//  {
//    // So enable them and try again
//    toggle_features();
//    write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
//  }
//
//  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
//
//  //
//  // Enable dynamic payload on pipes 0 & 1
//  //
//
//  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
//}
//
///****************************************************************************/
//
//void RF24::writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
//{
//  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);
//
//  csn(LOW);
//  SPI.transfer( W_ACK_PAYLOAD | ( pipe & B111 ) );
//  const uint8_t max_payload_size = 32;
//  uint8_t data_len = min(len,max_payload_size);
//  while ( data_len-- )
//    SPI.transfer(*current++);
//
//  csn(HIGH);
//}
//
///****************************************************************************/
//
//bool RF24::isAckPayloadAvailable(void)
//{
//  bool result = ack_payload_available;
//  ack_payload_available = false;
//  return result;
//}
//
///****************************************************************************/
//
//bool RF24::isPVariant(void)
//{
//  return p_variant ;
//}
//
///****************************************************************************/
//
//void RF24::setAutoAck(bool enable)
//{
//  if ( enable )
//    write_register(EN_AA, B111111);
//  else
//    write_register(EN_AA, 0);
//}
//
///****************************************************************************/
//
//void RF24::setAutoAck( uint8_t pipe, bool enable )
//{
//  if ( pipe <= 6 )
//  {
//    uint8_t en_aa = read_register( EN_AA ) ;
//    if( enable )
//    {
//      en_aa |= _BV(pipe) ;
//    }
//    else
//    {
//      en_aa &= ~_BV(pipe) ;
//    }
//    write_register( EN_AA, en_aa ) ;
//  }
//}
//
///****************************************************************************/
//
//bool RF24::testCarrier(void)
//{
//  return ( read_register(CD) & 1 );
//}
//
///****************************************************************************/
//
//bool RF24::testRPD(void)
//{
//  return ( read_register(RPD) & 1 ) ;
//}
//
///****************************************************************************/
//

void rf24_setPALevel(rf24_t *rf, rf24_pa_dbm_e level)
{
  uint8_t setup = rf24_read_register(rf, RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  rf24_write_register(rf, RF_SETUP, setup ) ;
}

///****************************************************************************/
//
//rf24_pa_dbm_e RF24::getPALevel(void)
//{
//  rf24_pa_dbm_e result = RF24_PA_ERROR ;
//  uint8_t power = read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
//
//  // switch uses RAM (evil!)
//  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
//  {
//    result = RF24_PA_MAX ;
//  }
//  else if ( power == _BV(RF_PWR_HIGH) )
//  {
//    result = RF24_PA_HIGH ;
//  }
//  else if ( power == _BV(RF_PWR_LOW) )
//  {
//    result = RF24_PA_LOW ;
//  }
//  else
//  {
//    result = RF24_PA_MIN ;
//  }
//
//  return result ;
//}

/****************************************************************************/

bool rf24_setDataRate(rf24_t *rf, rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = rf24_read_register(rf, RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  rf->wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    rf->wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      rf->wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      rf->wide_band = false ;
    }
  }
  rf24_write_register(rf, RF_SETUP,setup);

  // Verify our result
  if ( rf24_read_register(rf, RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    rf->wide_band = false;
  }

  return result;
}

///****************************************************************************/
//
//rf24_datarate_e RF24::getDataRate( void )
//{
//  rf24_datarate_e result ;
//  uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
//  
//  // switch uses RAM (evil!)
//  // Order matters in our case below
//  if ( dr == _BV(RF_DR_LOW) )
//  {
//    // '10' = 250KBPS
//    result = RF24_250KBPS ;
//  }
//  else if ( dr == _BV(RF_DR_HIGH) )
//  {
//    // '01' = 2MBPS
//    result = RF24_2MBPS ;
//  }
//  else
//  {
//    // '00' = 1MBPS
//    result = RF24_1MBPS ;
//  }
//  return result ;
//}

/****************************************************************************/

void rf24_setCRCLength(rf24_t *rf, rf24_crclength_e length)
{
  uint8_t config = rf24_read_register(rf, CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above. 
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  rf24_write_register(rf, CONFIG, config) ;
}

///****************************************************************************/
//
//rf24_crclength_e RF24::getCRCLength(void)
//{
//  rf24_crclength_e result = RF24_CRC_DISABLED;
//  uint8_t config = read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;
//
//  if ( config & _BV(EN_CRC ) )
//  {
//    if ( config & _BV(CRCO) )
//      result = RF24_CRC_16;
//    else
//      result = RF24_CRC_8;
//  }
//
//  return result;
//}

/****************************************************************************/

void rf24_disableCRC(rf24_t *rf)
{
  uint8_t disable = rf24_read_register(rf, CONFIG) & ~_BV(EN_CRC) ;
  rf24_write_register(rf, CONFIG, disable) ;
}

/****************************************************************************/

void rf24_setRetries(rf24_t *rf, uint8_t delay, uint8_t count)
{
    rf24_write_register(rf, SETUP_RETR, (delay & 0xf) << ARD | (count & 0xf) << ARC);
}

/****************************************************************************/

void rf24_setInterruptSources(rf24_t *rf, bool rx_dr, bool tx_ds, bool max_rt)
{
    uint8_t config = rf24_read_register (rf, CONFIG);

    config |= (_BV (MASK_RX_DR) | _BV (MASK_TX_DS) | _BV (MASK_MAX_RT));
    if (rx_dr)
        config &= ~_BV (MASK_RX_DR);
    if (tx_ds)
        config &= ~_BV (MASK_TX_DS);
    if (max_rt)
        config &= ~_BV (MASK_MAX_RT);

    rf24_write_register (rf, CONFIG, config);
}

/****************************************************************************/

void rf24_clearAllInterrupts(rf24_t *rf)
{
    rf24_write_register (rf, STATUS,
        rf24_read_register (rf, STATUS) | (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)) );
}

/****************************************************************************/

void rf24_disableAutoAck(rf24_t *rf)
{
    rf24_write_register(rf, EN_AA, 0);
}

/****************************************************************************/

void rf24_setAddressSize(rf24_t *rf, uint8_t size)
{
    uint8_t address_size;
    if (size == 0)
        address_size = 0;
    else
        address_size = max (3, min (5,size)) - 2;

    rf24_write_register(rf, SETUP_AW, address_size & 0x03);
}

/****************************************************************************/

void rf24_debugState(rf24_t *rf)
{
    volatile uint8_t status = rf24_get_status (rf);
    volatile uint8_t config = rf24_read_register (rf, CONFIG);
    volatile uint8_t en_aa = rf24_read_register (rf, EN_AA);
    volatile uint8_t en_rxaddr = rf24_read_register (rf, EN_RXADDR);
    volatile uint8_t setup_aw = rf24_read_register (rf, SETUP_AW);
    volatile uint8_t setup_retr = rf24_read_register (rf, SETUP_RETR);
    uint8_t tx_addr[5]; 
    rf24_read_register_buf (rf, TX_ADDR, tx_addr, 5);
    volatile uint8_t fifo = rf24_read_register (rf, FIFO_STATUS);
    volatile uint8_t feature = rf24_read_register (rf, FEATURE);
}

/****************************************************************************/

void rf24_setPowerState(bool power_on)
{
    if (power_on) {
        pio_config_set (EN_NRF_PIO, PIO_OUTPUT_HIGH);
        
        // prevent the ESD diodes powering the device through pull-up resistors
        // if this is not set then the device may not be properly reset
        pio_config_set (NRF_CSK_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_IRQ_PIO, PIO_INPUT);
        pio_config_set (NRF_MOSI_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_MISO_PIO, PIO_INPUT);
        pio_config_set (NRF_CE_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_CSN_PIO, PIO_OUTPUT_LOW);
    } else {
        pio_config_set (EN_NRF_PIO, PIO_OUTPUT_LOW);

        pio_config_set (NRF_CSK_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_IRQ_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_MOSI_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_MISO_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_CE_PIO, PIO_OUTPUT_LOW);
        pio_config_set (NRF_CSN_PIO, PIO_OUTPUT_LOW);
    }
    
}

