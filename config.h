#ifndef __CONFIG_H__
#define __CONFIG_H__

#define RTTY_CALLSIGN "CUBEX1"

/* CONFIGURABLE BITS */
#define ASCII 8          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 300     // Baud rate for use with RFM22B Max = 600
//#define RADIO_FREQUENCY 434550 // 434.550 Reality 
#define RADIO_POWER  0x07
#define RADIO_REBOOT 20  // Reboot Radio every X telemetry lines
#define APRS_TX_INTERVAL  120000  // APRS TX Interval 

#define TX_DELAY      300   // default was 300


#define SI446x_SHUTDOWN_PIN   5
#define SI446x_GPIO_PIN       6
#define SI446x_NIRQ_PIN       2

#define VCXO_FREQ 26000000L

#define RADIO_FREQUENCY   434460000UL


#define GPS_ENABLE_PIN        3


#define STATUS_LED            7  // Not Connected on CUBEX


#define CAM_TX_PIN            4
#define CAM_RX_PIN            8


#define VBATT_PIN             A0
#define VPANEL_PIN            A1
#define DIO1_PIN              A3
#define SDA_PIN               A4
#define SCL_PIN               A5

#endif

