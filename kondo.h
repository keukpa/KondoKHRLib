#ifndef __LIBKONDO_H__
#define __LIBKONDO_H__

#include <assert.h>
#include "ftdi.h"

typedef unsigned char UCHAR;	   // Define a UCHAR

// Servo Addresses
#define SERVO_START 0x90		// Address of SERVO 0
#define SERVO_SIZE  20			// 20 Bytes each
#define MAX_SERVO_ID 17			// Maximum of 35 servos
#define MAX_SERVOS 35			// Maximum number of servos

// RCB4 Commands
#define RCB4_ACK_BYTE   0x06            // Positive Ack Byte
#define RCB4_NCK_BYTE   0x15            // Negative Ack Byte
#define RCB4_CMD_ACK	0xFE 		//ACK Command

// USB Adapter
#define RCB4_BAUD	115200
#define RCB4_USB_VID	0x165c		// Vendor ID
#define RCB4_USB_PID	0x0007		// Part ID
#define RCB4_SWAP_SIZE	256
#define RCB4_OPT_BYTES	2

typedef struct
{
    struct ftdi_context ftdic;  // ftdi context
    UCHAR swap[RCB4_SWAP_SIZE];
    char error[128]; 		// error messages
    UCHAR options[RCB4_OPT_BYTES];	// option bytes
    int debug;
} KondoInstance;

typedef KondoInstance* KondoRef;


//declare general function
int kondo_create_instance( KondoRef ki ); 	// Create the KondoInstance object
int kondo_set_comms( KondoRef ki );	// Initialise the serial port
int kondo_close_instance( KondoRef ki );

//declare functions
int kondo_checksum(UCHAR OUTBUFFER[], int nBytes);		// Calulate checksum of Buffer
int kondo_ack( void );						// Send and receive an acknowledgment from RCB4
int kondo_send( KondoRef ki );					// Send data over serial port
int kondo_read( KondoRef ki, int bytesToRead ); 		// Receive Data
int kondo_purge_buffers( KondoRef ki );				// Purge the RCB4 buffers
int kondo_led(bool isLED);					// Toggle the LED on and off
int kondo_servo_ID_address(int SERVO_ID, bool SIO);		// Inform the EEPROM where the servo is in address space (by ID)
int kondo_servo_addresses( void);				// Inform the EEPROM where the servo is in address space (ALL)
int kondo_init_servo_ID();					// Which SERVOS do you wish to initialise?
int kondo_servo_free( void );					// FREE the servo (i.e. allow it to move)
int kondo_servo_hold( void );					// HOLD the servo (i.e. HOLD it in position)
int kondo_servo_on( void );					// Turn on all the activated servos
int kondo_set_servo_ID_pos( int IDX, int SIO, unsigned int aPos );
int kondo_read_servo_pos(int IDX);


#endif // __LIBKONDO_H__
