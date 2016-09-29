/*
 This file is part of eLeReS (by Michal Maciakowski - Cyberdrones.com).
 Adapted for MultiWii by Mis (Romuald Bialy)
 Ported to STM by Marbalon (Marcin Baliniak)
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>
#include "platform.h"

#include "build_config.h"
#include "debug.h"
#include "version.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/io.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "io/osd.h"

#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/eleres.h"

#include "telemetry/telemetry.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"


#include "io/beeper.h"
#include "drivers/bus_spi.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

/*****************    eLeReS compatibile reciver on RFM22B module  ********************/


static uint8_t red_led=0;
#define RED_LED_ON {if (!red_led) {rfm_spi_write(0x0e, 0x04);red_led=1;}}
#define RED_LED_OFF {if (red_led) {rfm_spi_write(0x0e, 0x00);red_led=0;}}

#ifdef RFM_SPI_CLK
    #define ENABLE_RFM        {spiSetDivisor(RFM_SPI, RFM_SPI_CLK);IOLo(rfmCsPin);}
#else
    #define ENABLE_RFM        IOLo(rfmCsPin)
#endif

#ifdef RFM_RESTORE_CLK
    #define DISABLE_RFM       {IOHi(rfmCsPin);spiSetDivisor(RFM_SPI, RFM_RESTORE_CLK);}
#else
    #define DISABLE_RFM       IOHi(rfmCsPin)
#endif
#define RC_CHANS 12


static IO_t rfmCsPin        = IO_NONE;
static IO_t rfmIrqPin        = IO_NONE;

//extern master_t masterConfig;
#define cfg masterConfig.eleresConfig

int16_t eleresData[RC_CHANS] = { 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};              // interval [1000;2000]
//eleres config

//#define RED_LED_ON {LED_on(&Stable_led);}
//#define RED_LED_OFF {LED_off(&Stable_led);}

void RF22B_init_parameter(void);
void to_rx_mode(void);
void ChannelHopping(uint8_t hops);
void telemetry_RX(void);
void rx_reset(void);
void to_tx_mode(uint8_t bytes_to_send);
void to_ready_mode(void);
void frequency_configurator(uint32_t frequency);

//static uint8_t rcChannel[] = {ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,AUX5,AUX6,AUX7,AUX8};
static uint32_t last_pack_time, next_pack_time, localizer_time;
static uint8_t DataReady, ch_hopping_time, first_run = 1, loc_force=0;

static uint8_t hop_list[16];
uint8_t RSSI = 0, QUALITY;
static uint16_t good_frames;
static volatile uint8_t RF_Mode;

uint8_t BkgLoc_enable = 0;
uint8_t BkgLoc_chlist;
uint8_t BkgLoc_buf[3][10];
uint8_t BkgLoc_cnt;

#define Transmit	1
#define Transmitted	2
#define Receive 	4
#define Received 	8
#define Preamble	16

#define RF22B_Rx_packet_received_interrupt  0x02
#define RF22B_PACKET_SENT_INTERRUPT         0x04
#define RF22B_VALID_PREAMBLE_INTERRUPT      0x40
#define RF22B_VALID_SYNCWORD_INTERRUPT  		0x80
#define DATA_PACKAGE_SIZE 22					// 12 bit per chanel + header
#define BIN_OFF_VALUE	1150
#define BIN_ON_VALUE	1850
uint8_t RF_Tx_Buffer[10];
uint8_t RF_Rx_Buffer[DATA_PACKAGE_SIZE];

uint8_t tx_full = 0;

//--------------------------------------------------------------
uint8_t rfm_spi_read(uint8_t address) {
    uint8_t byte;
    ENABLE_RFM;
    spiTransferByte(RFM_SPI, address & 0x7f);
    byte = spiTransferByte(RFM_SPI, 0x00);
    DISABLE_RFM;
    return byte;
}
//--------------------------------------------------------------
void rfm_spi_write(uint8_t address, uint8_t data) {
    ENABLE_RFM;
    spiTransferByte(RFM_SPI, address | 0x80);
    spiTransferByte(RFM_SPI, data);
    DISABLE_RFM;
}

uint8_t eleres_rssi(void)
{
    return RSSI;
}

//-----------------------------------------------------------------------
void rx_reset(void) {
  rfm_spi_write(0x07, 1);
  rfm_spi_write(0x08, 0x03);    //clear fifo disable multi packet
  rfm_spi_write(0x08, 0x00);    // clear fifo, disable multi packet
  rfm_spi_write(0x07, 5);  // to rx mode
  rfm_spi_write(0x05, RF22B_Rx_packet_received_interrupt);
  rfm_spi_write(0x06, RF22B_VALID_SYNCWORD_INTERRUPT);		// for RSSI reading
  rfm_spi_read(0x03);  //read the Interrupt Status1 register
  rfm_spi_read(0x04);
}
//-----------------------------------------------------------------------
void to_rx_mode(void) {
  to_ready_mode();
  rx_reset();
  RF_Mode = Receive;
}
//--------------------------------------------------------------
void to_tx_mode(uint8_t bytes_to_send) {
  uint8_t i;

  to_ready_mode();

  rfm_spi_write(0x08, 0x03);    // disABLE AUTO TX MODE, enable multi packet clear fifo
  rfm_spi_write(0x08, 0x00);    // disABLE AUTO TX MODE, enable multi packet, clear fifo

  rfm_spi_write(0x3e, bytes_to_send);    // bytes to send
  for (i = 0; i<bytes_to_send; i++)
    rfm_spi_write(0x7f, RF_Tx_Buffer[i]);

  rfm_spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);
  rfm_spi_write(0x06, 0);
  rfm_spi_read(0x03);      //read the Interrupt Status1 register
  rfm_spi_read(0x04);

  rfm_spi_write(0x07, 0x09);    // to tx mode
  RF_Mode = Transmit;
	tx_full = 0;
}
//--------------------------------------------------------------
void to_ready_mode(void) {
  rfm_spi_write(0x07, 1);
  rfm_spi_write(0x05, 0);
  rfm_spi_write(0x06, 0);
  rfm_spi_read(0x03);
  rfm_spi_read(0x04);
  RF_Mode = 0;
}
//--------------------------------------------------------------


//*****************************************************************************

static uint16_t eleresRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    if (chan >= rxRuntimeConfig->channelCount)
    {
        return 0;
    }

    return eleresData[chan];
}

bool eLeReS_control(void) {
  uint16_t rcData4Values[RC_CHANS];
  static uint32_t qtest_time, guard_time, led_time;
  static uint8_t rx_frames, loc_cnt;
  uint8_t channel_count;
  uint8_t i,n;
  uint16_t temp_int;
  uint32_t cr_time = millis();
  int red_led_local = 0;
  bool res = false;

	if (!IORead(rfmIrqPin))
    {
        Rfm_IRQ();
        return false; //we can't spend a lot of time here
    }

	//obsluga ledki....
	if (cr_time < (led_time + 500))
		red_led_local = 0;
	else if (cr_time < (led_time + 1000))
		red_led_local = 1;
	else
		led_time = cr_time;

  if((DataReady & 2) == 0) {
    if(cr_time > next_pack_time+2) {			// nie przyszla paczka w spodziewanym czasie + 3ms
      if ((cr_time-last_pack_time > 1500) || first_run) { // nie bylo paczki przez 1.5sek od ostatniej odebranej, lub pierwsze zalaczenie odbiornika
				RSSI = 18;
				RF22B_init_parameter();
        to_rx_mode();
        ChannelHopping(15);			         // skocz jeden kanal do tylu z listy
        next_pack_time += 17L*ch_hopping_time;             // nastepny skok bedzie po zrobieniu pelnego kanalu przez nadajnik
				if (cfg.eleres_telemetry_en)
				{
					telemetry_RX();
					to_tx_mode(9);
				}
      }
      else
			{								// zgubiono ramka, ale nie minelo 1.5 sekundy
        if(cr_time-last_pack_time > 3*ch_hopping_time)
        {	                // brak 3 ramek pod rzad
          red_led_local=1;
					if (RSSI > 0x18)
						RSSI--;
        }
        to_rx_mode();
        ChannelHopping(1);						// skocz do nastepnego kanal‚u w tabeli
        next_pack_time += ch_hopping_time;				// czekaj na kolejnea ramke przez czas "ch_hopping_time"
      }
    }
    if(cr_time > qtest_time) {
      qtest_time = cr_time + 500;        // quality test every 500ms
      QUALITY = good_frames * 100 / (500/ch_hopping_time);
      if(QUALITY > 100) QUALITY = 100;
      good_frames = 0;
    }
  }

	if((DataReady & 3) == 1) {
		if((DataReady & 4)==0) {
			channel_count = RF_Rx_Buffer[20] >> 4;
			if(channel_count < 4)  channel_count = 4;
			if(channel_count > RC_CHANS) channel_count = 12;
			for(i = 0; i<channel_count; i++) {
				temp_int = RF_Rx_Buffer[i+1];		// mlodsza czesc
				if(i%2 == 0)						              // kanaly 0,2,4,6...
					temp_int |= ((unsigned int)RF_Rx_Buffer[i/2 + 13] << 4) & 0x0F00;   // 4 bity ze starszej czesci bajtu
				else								      // kanaly 1,3,5,7...
					temp_int |= ((unsigned int)RF_Rx_Buffer[i/2 + 13] << 8) & 0x0F00;   // 4 bity z mlodszej czesci bajtu
				if ((temp_int>799) && (temp_int<2201)) rcData4Values[i] = temp_int;  // wartosc OK
			}
			n = RF_Rx_Buffer[19];							// odczyt kanalow binarnych
    for(i=channel_count; i < channel_count+5; i++) {	// doloz do kanalow z AP kanaly binarne
      if(i > 11) break;
      if(n & 0x01) temp_int = BIN_ON_VALUE;
      else temp_int = BIN_OFF_VALUE;
      rcData4Values[i] = temp_int;
      n >>= 1;
    }
    for(; i<RC_CHANS; i++) rcData4Values[i]=1500;     // unreceived channels fix
    // channel value filtering
		//to jest w computeRC
    for(i=0; i<RC_CHANS; i++) {
    //  temp_int = rcData4Values[rcChannel[cfg.rcmap[i]]];
      temp_int = rcData4Values[i];
      if (temp_int < eleresData[i] -3)  eleresData[i] = temp_int+2;
      if (temp_int > eleresData[i] +3)  eleresData[i] = temp_int-2;
    }

    //if(!rcOptions[BOXFAILSAFE]) failsafeCnt >>= 1; fix

    localizer_time = cr_time + (1000L*cfg.eleres_loc_delay);

		if (cfg.eleres_telemetry_en)
		{
			if (cfg.eleres_loc_en)
			{
				if(BkgLoc_enable == 0) BkgLoc_cnt=0;
				if(BkgLoc_cnt) BkgLoc_cnt--;

				if(BkgLoc_cnt<128)
					telemetry_RX();
				else
					memcpy(RF_Tx_Buffer, BkgLoc_buf[BkgLoc_cnt%3], 9);
			}
			else
				telemetry_RX();
		}

		if (cfg.eleres_telemetry_en)  	// jesli odblokowana telemetria
			telemetry_RX();
		}
		DataReady &= 0xFC;
		led_time = cr_time; //przyszla ramka wiec bede gasil lede

		res = true;
  }

	if (cfg.eleres_loc_en)
	{
		if((DataReady & 3)==3 && RF_Rx_Buffer[19]<128) {
			if(rx_frames == 0)	guard_time = last_pack_time;
			if(rx_frames < 250)	{rx_frames++;}
			if(rx_frames > 20 && cr_time-guard_time > (loc_force?5000:20000)) {
				DataReady = 0;
				localizer_time = cr_time + (1000L*cfg.eleres_loc_delay);
				RF22B_init_parameter();
				ChannelHopping(1);
				rx_frames = 0;

				if(loc_force && cfg.eleres_telemetry_en) {
					BkgLoc_enable = 1;
					temp_int = 0;
					for(i=0;i<16;i++) {
						uint16_t mult = hop_list[i] * hop_list[(i+1)%16];
						if(mult > temp_int) {temp_int = mult; BkgLoc_chlist = i;}
					}
				}
			}
		}

		if(cr_time-last_pack_time > 8000) {	// po 8 sekundach bez odbioru ramki resetuj licznik prawidlowych ramek
			rx_frames = 0;
		}
		if(!ARMING_FLAG(ARMED) && cr_time > localizer_time) {
			if((DataReady & 2)==0)
			{
				RF22B_init_parameter();
				rfm_spi_write(0x6d, cfg.eleres_loc_power);     // set localizer TX power
			}
			DataReady &= 0xFB;
			DataReady |= 2;
			localizer_time = cr_time+35;
			rfm_spi_write(0x79, 0);    // set channel 0
			red_led_local = 1;
			led_time = cr_time; //don't blink when localizer enabled

			BkgLoc_enable = 0;
			if(!(++loc_cnt & 1) && cfg.eleres_telemetry_en) {
				telemetry_RX();
				to_tx_mode(9);
			}
			else
			{
				RF_Tx_Buffer[0] = 0x48;
				RF_Tx_Buffer[1] = 0x45;
				RF_Tx_Buffer[2] = 0x4c;
				RF_Tx_Buffer[3] = 0x50;
				RF_Tx_Buffer[4] = 0x21;
				to_tx_mode(5);
			}
		}

  if((ARMING_FLAG(ARMED) || first_run) && (DataReady & 2)==0)
    localizer_time = cr_time + (1000L*cfg.eleres_loc_delay);

  if (cfg.eleres_telemetry_en)
    if(DataReady & 4) {
      if(RF_Rx_Buffer[0]=='H') BkgLoc_buf[0][0]='T';
      if(RF_Rx_Buffer[0]=='T') {BkgLoc_buf[0][0]='T'; memcpy(BkgLoc_buf[0]+2, RF_Rx_Buffer+2, 7); }
      if(RF_Rx_Buffer[0]=='P') memcpy(BkgLoc_buf[1], RF_Rx_Buffer, 9);
      if(RF_Rx_Buffer[0]=='G') memcpy(BkgLoc_buf[2], RF_Rx_Buffer, 9);
      DataReady = 0;
    }
  }

	if (red_led_local)
	{
		RED_LED_ON;
	}
	else
	{
		RED_LED_OFF;
	}
	return res;
}


//if (RF_Rx_Buffer[0]=='S' && RF_Rx_Buffer[19]<128)  // otrzymano servo control data i zezwo
void Finder_enable(void)
{
	uint16_t temp_int, i, mult;

	BkgLoc_enable = 1;
	temp_int = 0;
	for(i=0;i<16;i++) {
		mult = hop_list[i] * hop_list[(i+1)%16];
		if(mult > temp_int) {temp_int = mult; BkgLoc_chlist = i;}
	}
	cfg.eleres_telemetry_en = 1;
	cfg.eleres_loc_en = 1;
}

void GPS_distance_cm_bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);

int Finder_parse(tFinder_info *fi)
{
	union {int32_t val; uint8_t b[4];} cord_cnv[2];

	if (!BkgLoc_cnt)
		return 0;

	memcpy(cord_cnv, &BkgLoc_buf[1][1], 8);

	fi->coord[0] = cord_cnv[0].val * 10;
	fi->coord[1] = cord_cnv[1].val * 10;

	GPS_distance_cm_bearing(&GPS_coord[LAT], &GPS_coord[LON], &fi->coord[LAT], &fi->coord[LON], &fi->dist, &fi->dir);

	fi->dist /= 100;
	fi->dir /= 100;
	fi->vbat = BkgLoc_buf[0][3];
	fi->rssi = BkgLoc_buf[0][1] - 128;

	return 1;
}

/*
void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
			Rfm_IRQ();

	// Clear the EXTI line 4 pending bit
	EXTI_ClearITPendingBit(EXTI_Line4);
}
*/
//*****************************************************************************
//check if channel is already assigned, then choose another
uint8_t CheckChannel(uint8_t channel, uint8_t *hop_lst) {
  uint8_t new_channel, count = 0, high = 0, i;
  for (i=0; i<16; i++) {
    if (high<hop_lst[i]) high = hop_lst[i];
    if (channel==hop_lst[i]) count++;
  }
  if (count>0) new_channel = high+2;
  else new_channel = channel;
  return new_channel%255;
}

void Bind_Channels(uint8_t* RF_HEAD, uint8_t* hop_lst) {
  uint8_t n,i,j;
  for (i=0;i<16;i++) hop_lst[i] = 0;
  for (j=0;j<4;j++) {
    for (i=0;i<4;i++) {
      n = RF_HEAD[i]%128;
      if(j==3) n /= 5; else n /= j+1;
      hop_lst[4*j+i] = CheckChannel(n,hop_lst);
    }
  }
}

//*****************************************************************************

void Rfm_IRQ(void) {
  static uint16_t rssifil;
  uint32_t irq_time = millis();
  uint8_t St1,St2;

    St1 = rfm_spi_read(0x03);
	St2 = rfm_spi_read(0x04);
    spiTransferByte(RFM_SPI, 0x00);

    if((RF_Mode & Receive) && (St1 & RF22B_Rx_packet_received_interrupt))
      RF_Mode |= Received;
    if((RF_Mode & Transmit) && (St1 & RF22B_PACKET_SENT_INTERRUPT))
      RF_Mode |= Transmitted;
    if((RF_Mode & Receive) && (St2 & RF22B_VALID_SYNCWORD_INTERRUPT))
      RF_Mode |= Preamble;

    if(RF_Mode & Received) {   	// RFM22B INT pin Enabled by received Data
			uint8_t i;
				if(BkgLoc_enable < 2)
				{
					last_pack_time = irq_time;
					next_pack_time = irq_time + ch_hopping_time;
				}
			ENABLE_RFM;
			spiTransferByte(RFM_SPI, 0x7f);
			for(i = 0; i<DATA_PACKAGE_SIZE; i++)
				RF_Rx_Buffer[i] = spiTransferByte(RFM_SPI, 0x00);
			DISABLE_RFM;
			rx_reset();
			RF_Mode = Receive;
			if ((RF_Rx_Buffer[0] & 127) == 'S') {	// servo control data
				first_run = 0;
				good_frames++;
				ch_hopping_time = (RF_Rx_Buffer[20] & 0x0F)+18;		// szybkosc wysylania ramek w nadajniku w ms
				DataReady |= 1;
			}
			else if (cfg.eleres_loc_en && cfg.eleres_telemetry_en && BkgLoc_enable==2)
			{
				if((RF_Rx_Buffer[0] == 'H' && RF_Rx_Buffer[2] == 'L') ||
						RF_Rx_Buffer[0]=='T' || RF_Rx_Buffer[0]=='P' || RF_Rx_Buffer[0]=='G') {
					if(BkgLoc_cnt==0) BkgLoc_cnt = 200;
					to_ready_mode();
					BkgLoc_enable = 0;
					ChannelHopping(0);
					BkgLoc_enable = 2;
					DataReady |= 4;
				}
			}
			else if (cfg.eleres_loc_en)
			{
				if (RF_Rx_Buffer[0] == 0x4c && RF_Rx_Buffer[1] == 0x4f && RF_Rx_Buffer[2] == 0x43) {
					localizer_time = irq_time;
					loc_force = 1;
					RSSI = 0x18;
				}
			}

			if((DataReady & 2)==0) {
				if (cfg.eleres_telemetry_en)
					to_tx_mode(9);	        // send telemetry
				else
					ChannelHopping(1);      // standard hoping to next channel
			}
    }

    if(RF_Mode & Transmitted) {	// end of telemetry transmission, switch to receive and hop to next channel
      to_ready_mode();
      if(DataReady & 2) {
        rfm_spi_write(0x79, hop_list[0]);                  // first channel from list
      }
      else if(irq_time-last_pack_time <= 1500 && BkgLoc_enable<2)	        // jesli nie poszukiwanie sygnalu
        ChannelHopping(1);								// standard hoping to next channel
      to_rx_mode();
    }

    if(RF_Mode & Preamble) {			        // preamble detected, check RSSI
      uint8_t rssitmp = rfm_spi_read(0x26);
			if (cfg.eleres_loc_en && cfg.eleres_telemetry_en && BkgLoc_enable==2)
			{
        if(rssitmp>124)	rssitmp = 124;
        if(rssitmp<18)	rssitmp = 18;
        BkgLoc_buf[0][1] = rssitmp + 128;
			}
			else
			{
				rssifil -= rssifil/8;
				rssifil += rssitmp;
				RSSI = (rssifil/8 * QUALITY / 100)+10;
				if(RSSI>124)	RSSI = 124;
				if(RSSI<18)	RSSI = 18;
			}
			RF_Mode &= ~Preamble;
    }
}

//-------Defaults 38.400 baud----------------------------------------------
void RF22B_init_parameter(void) {
  int8_t i;
	static uint8_t first_init = 1;

  static uint8_t cf1[9] = {0x00,0x01,0x00,0x7f,0x07,0x52,0x55,0xCA};
  static uint8_t cf2[6] = {0x68,0x01,0x3a,0x93,0x02,0x6b};
  static uint8_t cf3[8] = {0x0f,0x42,0x07,0x20,0x2d,0xd4,0x00,0x00};
  rfm_spi_read(0x03);
  rfm_spi_read(0x04);
	for(i = 0; i < 8; i++)
		rfm_spi_write(0x06+i, cf1[i]);
	if (first_init)
	{
		first_init = 0;
		rfm_spi_write(0x0e, 0);
	}

  rfm_spi_write(0x6e, 0x09);
  rfm_spi_write(0x6f, 0xD5);
  rfm_spi_write(0x1c, 0x02);
  rfm_spi_write(0x70, 0x00);
  for(i=0; i<6; i++) rfm_spi_write(0x20+i, cf2[i]);
  rfm_spi_write(0x2a, 0x1e);
  rfm_spi_write(0x72, 0x1F);
  rfm_spi_write(0x30, 0x8c);
  rfm_spi_write(0x3e, 22);
  for(i=0; i<8; i++) rfm_spi_write(0x32+i, cf3[i]);
  for(i=0; i<4; i++) rfm_spi_write(0x43+i, 0xff);
  rfm_spi_write(0x6d, cfg.eleres_telemetry_power | 0x18);
  rfm_spi_write(0x79, 0x00);
  rfm_spi_write(0x7a, 0x04);
  rfm_spi_write(0x71, 0x23);
  rfm_spi_write(0x73, 0x00);
  rfm_spi_write(0x74, 0x00);
  for(i=0; i<4; i++) {
    rfm_spi_write(0x3a+i, cfg.eleres_signature[i]);
    rfm_spi_write(0x3f+i, cfg.eleres_signature[i]);
  }

  frequency_configurator(cfg.eleres_freq); // Calibrate the RFM22B to this frequency, frequency hopping starts from here.
  rfm_spi_read(0x03);
  rfm_spi_read(0x04);
}

void frequency_configurator(uint32_t frequency) {
  uint8_t band;
  // frequency formulation from Si4432 chip's datasheet
  if(frequency<48000) {
    frequency -= 24000;
    band = frequency/1000;				// calculate band [fb 4:0]
    if(band>23) band = 23;
    frequency -= (1000*(uint32_t)band);
    frequency *= 64;				 	// this is the Nominal Carrier Frequency (fc) value for register setting
    band |= 0x40;  				 	// select band LOW and SBSEL=1
  }
  else {
    frequency -= 48000;
    band = frequency/2000;				// calculate band [fb 4:0]
    if(band>22) band = 22;
    frequency -= (2000*(uint32_t)band);
    frequency *= 32;							// this is the Nominal Carrier Frequency (fc) value for register setting
    band |= 0x60;  					 	// select band HIGH and SBSEL=1
  }
  rfm_spi_write(0x75, band);    			// write band register
  rfm_spi_write(0x76, (uint16_t)frequency >> 8);    // write frequency
  rfm_spi_write(0x77, (uint8_t)frequency);
  rfm_spi_write(0x79, 0);    // set channel 0
}
//--------------------------------------------------------------
//how many channels to skip (ch = 1 <-just next channel)
void ChannelHopping(uint8_t hops) {
  static uint8_t hopping_channel = 1;
  hopping_channel += hops;
  while(hopping_channel >= 16) hopping_channel -= 16;

	if (cfg.eleres_telemetry_en && cfg.eleres_loc_en)
	{
		if(BkgLoc_enable && (hopping_channel==BkgLoc_chlist || hopping_channel==(BkgLoc_chlist+1)%16)) {
			rfm_spi_write(0x79,0);
			BkgLoc_enable = 2;
			return;
		}
		if(BkgLoc_enable == 2) BkgLoc_enable = 1;
	}

  rfm_spi_write(0x79, hop_list[hopping_channel]);
}
//*****************************************************************************

extern int32_t baroPressure;
extern int32_t baroTemperature;

void telemetry_RX(void) {
	static uint8_t telem_state;
	static int32_t presfil;
	static int16_t thempfil;
	uint8_t i, themp=90, wii_flymode=0;
	uint16_t pres,curr = 0;  //uint16_t pres,curr = mA /100; fix
	union {int32_t val; uint8_t b[4];} cnv;

	if (tx_full)
		return;

	memset(RF_Tx_Buffer,0,9);

	presfil  -= presfil/4;
	presfil  += baroPressure;
	thempfil -= thempfil/8;
	thempfil += baroTemperature/10;

	switch (telem_state++)
	{
		case 0:

			if(presfil>200000) pres = presfil/4 - 50000;
			else pres = 1;

			themp = (uint8_t)(thempfil/80 + 86);// przeliczenie i obciecie do 1 stopnia

			if (FLIGHT_MODE(FAILSAFE_MODE))    wii_flymode = 7;
			else if(FLIGHT_MODE(PASSTHRU_MODE))  wii_flymode = 8;
			else if(FLIGHT_MODE(GPS_HOME_MODE))  wii_flymode = 6;
			else if(FLIGHT_MODE(GPS_HOLD_MODE))  wii_flymode = 5;
			else if(FLIGHT_MODE(HEADFREE_MODE))  wii_flymode = 4;
			else if(FLIGHT_MODE(BARO_MODE))      wii_flymode = 3;
			else if(FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE))       wii_flymode = 2;
			else                      wii_flymode = 1;
			if(ARMING_FLAG(ARMED)) wii_flymode |= 0x10;
			RF_Tx_Buffer[0] = 0x54;			  // Telemetry data
			RF_Tx_Buffer[1] = RSSI;			  // 16-134
			RF_Tx_Buffer[2] = QUALITY;		  // 0-100
			RF_Tx_Buffer[3] = vbat;		          // napieciÄ™ z Wii w 0.1V
			RF_Tx_Buffer[4] = themp;                    // temperatura
			RF_Tx_Buffer[5] = curr & 0xff;                        // ADC z modulu RFM22B (0)
			RF_Tx_Buffer[6] = pres>>8;                  // cisnienie z baro minus 50000
			RF_Tx_Buffer[7] = pres&0xFF;
			RF_Tx_Buffer[8] = wii_flymode;	          // tryb lotu multiwii
			RF_Tx_Buffer[8]	|= ((curr>>2) & 0xC0);	// plus starsze dwa bity pradu
		break;

	case 1:
		if (GPS_coord[LAT]) {
			RF_Tx_Buffer[0] = 0x50;                      // GPS position
			cnv.val = GPS_coord[LAT]/10;
			for(i=0;i<4;i++) RF_Tx_Buffer[i+1] = cnv.b[i];
			cnv.val = GPS_coord[LON]/10;
			for(i=0;i<4;i++) RF_Tx_Buffer[i+5] = cnv.b[i];
			break;
		}
		telem_state++;
	case 2:
		if (sensors(SENSOR_GPS)) {
			uint16_t gpsspeed =  (GPS_speed*9L)/250L;
			int16_t course = (GPS_ground_course+360)%360;
			uint16_t tim = 0; /*= 10*hex_c(GPS_time[0]) + hex_c(GPS_time[1]);
			tim <<= 11;
			i = 10*hex_c(GPS_time[2]) + hex_c(GPS_time[3]);
			tim |= (uint16_t)i<<5;
			i = 10*hex_c(GPS_time[4]) + hex_c(GPS_time[5]);
			tim |= (i>>1); fix */

			RF_Tx_Buffer[0] = 0x47;				// other GPS data
			RF_Tx_Buffer[1] = (STATE(GPS_FIX)<<4) | (GPS_numSat & 0x0F);
			if(GPS_numSat > 15) RF_Tx_Buffer[1] |= 0x80;
			RF_Tx_Buffer[2] = ((course>>8) & 0x0F) | ((gpsspeed>>4) & 0xF0);
			RF_Tx_Buffer[3] = course & 0xFF;
			RF_Tx_Buffer[4] = gpsspeed & 0xFF;

			RF_Tx_Buffer[5] = (GPS_altitude/100) >>8;
			RF_Tx_Buffer[6] = (GPS_altitude/100) & 0xFF;

			RF_Tx_Buffer[7] = tim>>8;
			RF_Tx_Buffer[8] = tim&0xFF;
			break;
		}
		telem_state++;
	default:
	  RF_Tx_Buffer[0] = 'D';				// 1 data
	  memcpy(RF_Tx_Buffer+1,&debug[0],2);
	  memcpy(RF_Tx_Buffer+3,&debug[1],2);
	  memcpy(RF_Tx_Buffer+5,&debug[2],2);
	  memcpy(RF_Tx_Buffer+7,&debug[3],2);
		telem_state = 0;
	break;
	}
	tx_full = 1;
}

bool eleresInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback) {
    UNUSED(rxConfig);

    rfmCsPin = IOGetByTag(IO_TAG(RFM_SPI_CS_PIN));
    IOInit(rfmCsPin, OWNER_RX, RESOURCE_SPI_CS, 0);
    IOConfigGPIO(rfmCsPin, SPI_IO_CS_CFG);

    rfmIrqPin = IOGetByTag(IO_TAG(RFM_IRQ_PIN));
    IOInit(rfmIrqPin, OWNER_RX, RESOURCE_NONE, 0);
    IOConfigGPIO(rfmIrqPin, IOCFG_IN_FLOATING);

    spiSetDivisor(MAX7456_SPI_INSTANCE, SPI_CLOCK_STANDARD);


    if (cfg.eleres_freq < 41500 || cfg.eleres_freq > 45000)
    {
        cfg.eleres_freq = 43500;
        cfg.eleres_telemetry_en = 0;
        cfg.eleres_telemetry_power = 7;
        cfg.eleres_loc_en = 0;
        cfg.eleres_loc_power = 7;
        cfg.eleres_loc_delay = 240;
    }


    rxRuntimeConfig->channelCount = RC_CHANS;
    if (callback)
        *callback = eleresRawRC;

  DISABLE_RFM;

  //Tu zainicjowaæ przerwania itp.
  /*GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
  EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/

	//software reset
  rfm_spi_write(0x07, 0x80);    // software reset
  delay(100);

  RF22B_init_parameter();
  Bind_Channels(cfg.eleres_signature,hop_list);
  ch_hopping_time = 33;		// Startup at maximum Hoping Time
  to_rx_mode();
  ChannelHopping(1);
  RF_Mode = Receive;
    localizer_time = millis() + (1000L * cfg.eleres_loc_delay);

    return true;
}

uint8_t eLeReS_Bind(void) {
  static uint8_t eleres_signature_old[4];
  static uint8_t eleres_signature_OK_count = 0;
  uint16_t timeout = 10000;
  uint8_t i;
  cfg.eleres_signature[0] = 0x42;
  cfg.eleres_signature[1] = 0x49;
  cfg.eleres_signature[2] = 0x4e;
  cfg.eleres_signature[3] = 0x44;
  RF22B_init_parameter();
  Bind_Channels(cfg.eleres_signature,hop_list);
  ch_hopping_time = 33;
  RED_LED_OFF;
  while(timeout--) {
    eLeReS_control();
    if (RF_Rx_Buffer[0]==0x42)
    {
      for(i=0; i<4; i++) {
        if (RF_Rx_Buffer[i+1]==eleres_signature_old[i]) eleres_signature_OK_count++; else eleres_signature_OK_count = 0;
      }
      for(i=0; i<4; i++) eleres_signature_old[i] = RF_Rx_Buffer[i+1];
      if (eleres_signature_OK_count>200) {
        for(i=0; i<4; i++)
					cfg.eleres_signature[i] = eleres_signature_old[i];
        //zapisac sygnature
				RED_LED_OFF;

				saveConfigAndNotify();
				RF22B_init_parameter();
        return 0;
      }
      RF_Rx_Buffer[0] = 0;
    }
	delay(1);
  }
  RF22B_init_parameter();
  return 1; //timeout
}

