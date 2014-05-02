/*
 CUBEX V1
 By: Arko
 
 FUSE SETTINGS: EXTENDED 0xFD
                HIGH     0xD8
                LOW      0xE2  (Internal 8Mhz osc)
 
 AVA 70cms Tracker
 
 By Anthony Stirk M0UPU 
 
 October 2012 Version 3
 Subversion 3.35 FLIGHT READY
 
 Thanks and credits :
 
 Interrupt Driven RTTY Code :
 Evolved from Rob Harrison's RTTY Code.
 Thanks to : 
 http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/
 http://gammon.com.au/power
 Suggestion to use Frequency Shift Registers by Dave Akerman (Daveake)/Richard Cresswell (Navrac)
 Suggestion to lock variables when making the telemetry string & Compare match register calculation from Phil Heron.
 
 RFM22B Code from James Coxon http://ukhas.org.uk/guides:rfm22b 
 
 GPS Code from jonsowman and Joey flight computer CUSF
 https://github.com/cuspaceflight/joey-m/tree/master/firmware
 Big thanks to Dave Akerman!
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 See <http://www.gnu.org/licenses/>.
 */
/*
CODE FIXES: - GPS padding error has been fixed in this code
*/

#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#define F_CPU (8000000)

#include "config.h"
#include "radio_si446x.h"
#include "rtty.h"
#include "ssdv.h"


#include <Adafruit_VC0706.h>
#include <SoftwareSerial.h>  

// CAM_TX: 4, CAM_RX: 8
SoftwareSerial cameraconnection = SoftwareSerial(CAM_TX_PIN, CAM_RX_PIN);
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);
unsigned char cameracode = 0x00;


//#define POWERSAVING      // Comment out to turn power saving off

uint8_t buf[60]; 
char txstring[80];
volatile int txstatus=1;
volatile int txstringlength=0;
volatile char txc;
volatile int txi;
volatile int txj;
volatile int count=1;
volatile boolean lockvariables = 0;
uint8_t lock =0, sats = 0, hour = 0, minute = 0, second = 0;
uint8_t oldhour = 0, oldminute = 0, oldsecond = 0;
int navmode = 0, battv=0, rawbattv=0, GPSerror = 0, lat_int=0,lon_int=0,txnulls=10;
int32_t lat = 0, lon = 0, alt = 0, maxalt = 0, lat_dec = 0, lon_dec =0, battvaverage=0;
int psm_status = 0, radiostatus=0, countreset=0, aprs_attempts=0, aprs_tx_status=0;
unsigned long aprs_startTime;
int32_t tslf=0;
int errorstatus=0; 
/* Bit 0 = GPS Error Condition Noted Switch to Max Performance Mode
 Bit 1 = GPS Error Condition Noted Cold Boot GPS
 Bit 2 = RFM22B Error Condition Noted, RFM22B Power Cycled
 Bit 3 = Current Dynamic Model 0 = Flight 1 = Pedestrian
 Bit 4 = PSM Status 0 = PSM On 1 = PSM Off                   
 Bit 5 = Lock 0 = GPS Locked 1= Not Locked
 */



void setup() {
  // Initializes the camera to run at 9600baud. Once this code is executed, you will need
  // cycle the power for it to take effect. You only need this code to run once really, but
  // since we will likely build more of these, we leave this in here for new hardware.
    delay(1000);
    cam.begin(38400);  // Cameras already set to 9600baud will ignore this..
    delay(1000);
    cam.setbaudlow();  // Set the camera to low baud rate (9600) if you haven't already
  
  // Setup Camera  
  delay(1000);
  if (cam.begin(9600)) 
  {
    cameracode = 1;
  } 
  else 
  {
    cameracode = 2;
  }
  delay(1000);
  
  cam.setImageSize(VC0706_320x240);

  uint8_t imgsize = cam.getImageSize();
  /*
  if (!cam.takePicture()) 
  {
    cameracode = 20; // Camera fail to snap picture
  }
  else 
  {
    cameracode = 10; // Success!
  }
  
  uint16_t jpglen = cam.frameLength();
  */
  
  
  //Setup
  pinMode(STATUS_LED, OUTPUT); 
  
  pinMode(GPS_ENABLE_PIN,OUTPUT);
  digitalWrite(GPS_ENABLE_PIN,LOW);
  
  pinMode(SI446x_GPIO_PIN, OUTPUT);
  digitalWrite(SI446x_GPIO_PIN, LOW);
  
  // THE FOLLOWING PINS HAVE NOT YET BEEN CONFIGURED!!
    // START
    //pinMode(CAM_TX_PIN, INPUT); 
    //pinMode(CAM_RX_PIN, INPUT);
    pinMode(VBATT_PIN, INPUT);
    pinMode(VPANEL_PIN, INPUT);
    pinMode(DIO1_PIN, INPUT);
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    //END
  
  
  blinkled(6);
  Serial.begin(9600);    // GPS Serial Communication (RX/TX Pins)
  blinkled(5);
  resetGPS();
  blinkled(4);
  wait(500);
  blinkled(3);
  
  // Start the radio
  pinMode(SI446x_SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SI446x_SHUTDOWN_PIN, LOW);

  startup();
  ptt_on();

  digitalWrite(SI446x_GPIO_PIN, HIGH);
  blinkled(2);
  setupGPS();
  blinkled(1);
  //initialise_interrupt();  // This is the old TIMER0 interrupt
  
#ifdef POWERSAVING
  ADCSRA = 0;
#endif
  
  rtx_init();    // Setup RTTY/Interrupt
}

void loop()
{
  delay(10);
  
  oldhour=hour;
  oldminute=minute;
  oldsecond=second;
  gps_check_nav();

  if(lock!=3) // Blink LED to indicate no lock
  {
    digitalWrite(STATUS_LED, HIGH);   
    wait(750);               
    digitalWrite(STATUS_LED, LOW); 
    errorstatus |=(1 << 5);     
  }
  else
  {
    errorstatus &= ~(1 << 5);
  }
  checkDynamicModel();
#ifdef POWERSAVING
  if((lock==3) && (psm_status==0) && (sats>=5) &&((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))
  {
    setGPS_PowerSaveMode();
    wait(1000);
    pinMode(STATUS_LED, INPUT); 
    psm_status=1;
    errorstatus &= ~(1 << 4);
  }
#endif

  if(!lockvariables) {

    prepare_data();
    if(alt>maxalt && sats >= 4)
    {
      maxalt=alt;
    }

    if((oldhour==hour&&oldminute==minute&&oldsecond==second)||sats<=4) {
      tslf++;
    }
    else
    {
      tslf=0;
      errorstatus &= ~(1 << 0);
      errorstatus &= ~(1 << 1);
    }
    if((tslf>10 && ((errorstatus & (1 << 0))==0)&&((errorstatus & (1 << 1))==0))) {
      setupGPS();
      wait(125);
      setGps_MaxPerformanceMode();
      wait(125);
      //    errorstatus=1;
      errorstatus |=(1 << 0);
      psm_status=0;
      errorstatus |=(1 << 4); 
    }
    if(tslf>100 && ((errorstatus & (1 << 0))==1)&&((errorstatus & (1 << 1))==0)) {
      errorstatus |=(1 << 0);
      errorstatus |=(1 << 1);
      Serial.flush();
      resetGPS();
      wait(125);
      setupGPS();
    }
  }
  
  //rtx_string_P(PSTR("CUBEX1 - loop\n"));
  
  if(alt>maxalt && sats >= 4)
  {
    maxalt=alt;
  }
  lockvariables=1;
  
  
  sprintf_P(txstring, PSTR("$$$$$CUBEX1,%i,%02d:%02d:%02d,%s%i.%05ld,%s%i.%05ld,%ld,%d,%i,%i"),count, hour, minute, second,lat < 0 ? "-" : "",lat_int,lat_dec,lon < 0 ? "-" : "",lon_int,lon_dec, maxalt,sats,errorstatus,cameracode);

  sprintf_P(txstring, PSTR("%s*%04X\n"), txstring, gps_CRC16_checksum(txstring));
  maxalt=0;
  lockvariables=0;
  txstringlength=strlen(txstring);
  rtx_string(txstring);
  count++;
  char testvars = tx_image();
}    

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  PROGMEM static const uint8_t setNMEAoff[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9
  };
  sendUBX_P(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  wait(500);
  setGPS_DynamicModel6();
  wait(500);
  setGps_MaxPerformanceMode();
  wait(500);
}

void sendUBX_P(const uint8_t *msg, uint8_t len) {
  Serial.flush();
  Serial.write(0xFF);
  wait(100);
  while(len--) {
    Serial.write(pgm_read_byte(msg++));
  }
}

uint8_t gps_check_nav(void)
{
  PROGMEM static const uint8_t request[8] = {
    0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84
  };
  sendUBX_P(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify sync and header bytes
  if( buf[0] != 0xB5 || buf[1] != 0x62 ){
    GPSerror = 41;
  }
  if( buf[2] != 0x06 || buf[3] != 0x24 ){
    GPSerror = 42;
  }
  // Check 40 bytes of message checksum
  if( !_gps_verify_checksum(&buf[2], 40) ) {
    GPSerror = 43;
  }


  // Return the navigation mode and let the caller analyse it
  navmode = buf[8];
}
void gps_get_data()
{
  Serial.flush();
  // Clear buf[i]
  for(int i = 0;i<60;i++) 
  {
    buf[i] = 0; // clearing buffer  
  }  
  int i = 0;
  unsigned long startTime = millis();

  while ((i<60) && ((millis() - startTime) < 1000) ) { 
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
  }
}
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
  uint8_t a, b;
  gps_ubx_checksum(data, len, &a, &b);
  if( a != *(data + len) || b != *(data + len + 1))
    return false;
  else
    return true;
}
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
uint8_t* ckb)
{
  *cka = 0;
  *ckb = 0;
  for( uint8_t i = 0; i < len; i++ )
  {
    *cka += *data;
    *ckb += *cka;
    data++;
  }
}
boolean getUBX_ACK_P(const uint8_t *msg) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = pgm_read_byte(&msg[2]);	// ACK class
  ackPacket[7] = pgm_read_byte(&msg[3]);	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t ubxi=2; ubxi<8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      return false;
    }

    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }

    }
  }
}
void gps_check_lock()
{
  GPSerror = 0;
  Serial.flush();
  // Construct the request to the GPS
  PROGMEM static const uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
    0x07, 0x16
  };
  sendUBX_P(request, 8);

  // Get the message back from the GPS
  gps_get_data();
  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
    GPSerror = 11;
  }
  if( buf[2] != 0x01 || buf[3] != 0x06 ) {
    GPSerror = 12;
  }

  // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
  if( !_gps_verify_checksum(&buf[2], 56) ) {
    GPSerror = 13;
  }

  if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
      lock = buf[16];
    else
      lock = 0;

    sats = buf[53];
  }
  else {
    lock = 0;
  }
}

void setGPS_DynamicModel6()
{
  int gps_set_sucess=0;
  PROGMEM static const uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };
  while(!gps_set_sucess)
  {
    sendUBX_P(setdm6, sizeof(setdm6)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK_P(setdm6);
  }
}

void setGPS_DynamicModel3()
{
  int gps_set_sucess=0;
  PROGMEM static const uint8_t setdm3[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76
  };
  while(!gps_set_sucess)
  {
    sendUBX_P(setdm3, sizeof(setdm3)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK_P(setdm3);
  }
}
void gps_get_position()
{
  GPSerror = 0;
  Serial.flush();
  // Request a NAV-POSLLH message from the GPS
  PROGMEM static const uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
    0x0A
  };
  sendUBX_P(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 21;
  if( buf[2] != 0x01 || buf[3] != 0x02 )
    GPSerror = 22;

  if( !_gps_verify_checksum(&buf[2], 32) ) {
    GPSerror = 23;
  }

  if(GPSerror == 0) {
    if(sats<4)
    {
      lat=0;
      lon=0;
      alt=0;
    }
    else
    {
      lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
        (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
      lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
        (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
      alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
        (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
    }
    // 4 bytes of latitude/longitude (1e-7)
    lon_int=abs(lon/10000000);
    lon_dec=(labs(lon) % 10000000)/100;
    lat_int=abs(lat/10000000);
    lat_dec=(labs(lat) % 10000000)/100;


    // 4 bytes of altitude above MSL (mm)

    alt /= 1000; // Correct to meters
  }

}
void gps_get_time()
{
  GPSerror = 0;
  Serial.flush();
  // Send a NAV-TIMEUTC message to the receiver
  PROGMEM static const uint8_t request[8] = {
    0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
    0x22, 0x67
  };
  sendUBX_P(request, 8);

  // Get the message back from the GPS
  gps_get_data();

  // Verify the sync and header bits
  if( buf[0] != 0xB5 || buf[1] != 0x62 )
    GPSerror = 31;
  if( buf[2] != 0x01 || buf[3] != 0x21 )
    GPSerror = 32;

  if( !_gps_verify_checksum(&buf[2], 24) ) {
    GPSerror = 33;
  }

  if(GPSerror == 0) {
    if(buf[22] > 23 || buf[23] > 59 || buf[24] > 59)
    {
      GPSerror = 34;
    }
    else {
      hour = buf[22];
      minute = buf[23];
      second = buf[24];
    }
  }
}
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 5; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

void setGPS_PowerSaveMode() {
  // Power Save Mode 
  PROGMEM static const uint8_t setPSM[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92
  }; // Setup for Power Save Mode (Default Cyclic 1s)
  sendUBX_P(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

void setGps_MaxPerformanceMode() {
  //Set GPS for Max Performance Mode
  PROGMEM static const uint8_t setMax[] = { 
    0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91
  }; // Setup for Max Power Mode
  sendUBX_P(setMax, sizeof(setMax)/sizeof(uint8_t));
}
void resetGPS() {
  PROGMEM static const uint8_t set_reset[] = {
    0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5
  };
  sendUBX_P(set_reset, sizeof(set_reset)/sizeof(uint8_t));
}

void prepare_data() {

  gps_check_lock();
  gps_get_position();
  gps_get_time();
}

void blinkled(int blinks)
{
  for(int blinkledx = 0; blinkledx <= blinks; blinkledx++) {
    digitalWrite(STATUS_LED,HIGH);
    wait(100);
    digitalWrite(STATUS_LED,LOW);
    wait(100);
  }    
}    

void wait(unsigned long delaytime) // Arduino Delay doesn't get CPU Speeds below 8Mhz
{
  unsigned long _delaytime=millis();
  while((_delaytime+delaytime)>=millis()){
  }
}



void checkDynamicModel() {
  if(alt<=1000&&sats>4) {
    if(navmode != 3)
    {
      setGPS_DynamicModel3();
      errorstatus |=(1 << 3);      
    }
  }
  else
  {
    if(navmode != 6){
      setGPS_DynamicModel6();
      errorstatus &= ~(1 << 3);

    }
  }
}

char tx_image(void)
{
	static char setup = 0;
	static uint8_t img_id = 0;
	static ssdv_t ssdv;
	static uint8_t pkt[SSDV_PKT_SIZE];
	//static uint8_t img[64];
	int r;
	
      
        static uint8_t imgsize; 
        static uint16_t jpglen; 
        
	
	
        if(!setup)
	{
          setup = -1;
          cam.setImageSize(VC0706_320x240);
		
          imgsize = cam.getImageSize();
          cam.takePicture();
          jpglen = cam.frameLength();
          
          errorstatus = sizeof(ssdv_t);
        
          ssdv_enc_init(&ssdv, RTTY_CALLSIGN, img_id++);
          ssdv_enc_set_buffer(&ssdv, pkt);                  
	}

	while((r = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
	{
                uint8_t *buffer; 
                uint8_t bytesToRead = min(64, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
                buffer = cam.readPicture(bytesToRead);
                  
		//size_t r = c3_read(img, 64);
		//if(r == 0) break;
		
		ssdv_enc_feed(&ssdv, buffer, bytesToRead);
                jpglen -= bytesToRead;
	}
	
	if(r != SSDV_OK)
	{
		// Something went wrong! //
		//c3_close();
		//setup = 0;
	        //rtx_string_P(PSTR("$$" RTTY_CALLSIGN ":ssdv_enc_get_packet() failed again\n"));
                //snprintf_P((char *) img, 64, PSTR("$$" RTTY_CALLSIGN ":Camera error %d\n"), r);
		//rtx_string((char *) img);
		//return(setup);
	}
	
	if(!(jpglen > 0))
	{
		// The end of the image has been reached //
		//c3_close();
		setup = 0;
                cam.resumeVideo();
	}
	
	// Got the packet! Transmit it //
	rtx_data(pkt, SSDV_PKT_SIZE);
	
	return(setup);
}





























