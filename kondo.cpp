#include "kondo.h"

#include <stdlib.h>
#include <stdio.h>

unsigned char INBUFFER[256];
unsigned char OUTBUFFER[256];
unsigned char SERVOS[35];

int bStatus;
int returnVal = 0; // Used for the return value of functions

int result = 0;

unsigned int FREE_POS = 32768;
unsigned int HOLD_POS = 32767;


KondoInstance ki;

bool debug = 0;

struct ftdi_context *ftdi;

int kondo_create_instance(KondoRef ki)
{
  kondo_set_comms(ki);
  return 0;
}

/*
 * Function to connect to USB
 *
 * This should use FTDI (Hopefully)
 */
int kondo_set_comms( KondoRef ki )
{
  assert(ki);
  int i;

  ki->debug = 1;
  debug = 1;

  if(ftdi_init(&ki->ftdic) < 0)
    printf("Some Error\n");

  if (ftdi_set_interface(&ki->ftdic, INTERFACE_ANY) < 0)
    printf("Some Interface Error\n");

  if(ftdi_usb_open(&ki->ftdic, RCB4_USB_VID, RCB4_USB_PID) < 0)
    printf("Some USB opening error\n");

  if(ftdi_set_baudrate(&ki->ftdic, RCB4_BAUD) < 0)
    printf("Error with boad rate\n");

  if(ftdi_set_line_property(&ki->ftdic, BITS_8, STOP_BIT_1, EVEN) < 0)
    printf("Error setting line parameters\n");

  return 0;
}

int kondo_close_instance( KondoRef ki )
{
  assert(ki);

  if(ftdi_usb_close(&ki->ftdic) < 0)
    printf("Error closing connection\n");

  ftdi_deinit(&ki->ftdic);

  return 0;
}


/*
 * Function kondo_checksum - returns the sum on n-1 bytes of the OUTBUFFER[]
 * This function only needs to return the lower byte of the checksum.
 * @param UCHAR OUTBUFFER[] - The array of byte codes to send to the RCB4
 * @param int nBytes - number of bytes from OUTBUFFER to sum
 */
int kondo_checksum(UCHAR OUTBUFFER[], int nBytes)
{
  unsigned int sum = 0;
  // loop and add together bytes beig sent
  for(int i = 0; i < nBytes; i++)
  {
    sum += OUTBUFFER[i];
  }
  return sum;
}


/*
 * Function kondo_ack - send and receives an acknowledgement from the RCB4
 */
int kondo_ack()
{
  OUTBUFFER[0] = 4;
  OUTBUFFER[1] = RCB4_CMD_ACK;
  OUTBUFFER[2] = RCB4_ACK_BYTE;
  OUTBUFFER[3] = kondo_checksum(OUTBUFFER, 3);

  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  if(INBUFFER[2] == RCB4_ACK_BYTE)
  {
    return returnVal; // All OK
  }
  else if(INBUFFER[2] == RCB4_NCK_BYTE)
  {
    return -1; // Non-ack but RCB4 present
  }
  else
  {
    return -2; // No RCB4 connected
  }
}


/*
 * Function kondo_send writes data from the OUTBUFFER buffer to the ftdi device
 * stored in KondoReference struct
 *
 * @param KondoRef ki - a KondoRef struct
 */
int kondo_send(KondoRef ki)
{
  assert(ki);
  kondo_purge_buffers(ki);
  int i;

  if((i = ftdi_write_data(&ki->ftdic, OUTBUFFER, OUTBUFFER[0])) < 0)
    printf("Some error in writing to FTDI\n");

  if (&ki->debug)
  {
    printf("DEBUG: value of i is %d \n", i);
  }

  return i;
}

/*
 * Function kondo_read receives data from the INBUFFER buffer from the ftdi device
 * stored in KondoReference struct
 *
 * @param KondoRef ki - a KondoRef struct
 * @param int numBytes - how many bytes to read from the INBUFFER buffer
 */
int kondo_read(KondoRef ki, int numBytes)
{

  assert(ki);
  int i;

  if((i = ftdi_read_data(&ki->ftdic, INBUFFER, numBytes)) < 0)
  {
    printf("Some error in reading from FTDI\n");
  }

  if(&ki->debug)
  {
    printf("Debug Received: ");
    for(int i = 0; i < numBytes; i++)
    {
      printf("%X ", INBUFFER[i]);
    }
    printf("\n");
  }

  return i;
}

/* Function kondo_purge_buffers is used to purge both the TX and RB buffers
 *
 * @param KondoRef ki - a KondoRef struct
 * Returns 0 if sucessful <0 if error.
 *
 */
int kondo_purge_buffers(KondoRef ki)
{
  assert(ki);
  if (ftdi_usb_purge_buffers(&ki->ftdic) < 0)
    printf("Error when purging buffers.\n");

  return 0;
}

/*
 * Function kondo_led switches on/off the RCB4 Green LED
 * @param bool isLED - a boolean value true = on; false = off
 */
int kondo_led(bool isLED)
{
  int light_on = 0;

  if(isLED)
  {
    light_on = 128;
  }

  OUTBUFFER[0] = 0x09;
  OUTBUFFER[1] = 0x00;
  OUTBUFFER[2] = 0x02;
  OUTBUFFER[3] = 0x00;
  OUTBUFFER[4] = 0x00;
  OUTBUFFER[5] = 0x00;
  OUTBUFFER[6] = 0x31;
  OUTBUFFER[7] = light_on;
  OUTBUFFER[8] = kondo_checksum(OUTBUFFER, 8);

  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  return returnVal;
}


/*
 * Function kondo_servo_ID_address - Sets a single servo address
 * @param int Servo_ID - the ID number of the servo
 * @param bool SIO - which SIO channel (i.e. false = lower, true = upper)
 *
 */
int kondo_servo_ID_address(int Servo_ID, bool SIO)
{
  unsigned int servo_address = (Servo_ID * 2) * SERVO_SIZE + SERVO_START;

  if(SIO)
  {
    servo_address += 20;
  }

  unsigned int ADDH = (unsigned int) servo_address >> 8;
  unsigned int ADDL = (unsigned int) servo_address & 0x00FF;

  OUTBUFFER[0] = 0x1B;
  OUTBUFFER[1] = 0x00;
  OUTBUFFER[2] = 0x02;
  OUTBUFFER[3] = ADDL;		// Address L
  OUTBUFFER[4] = ADDH;		// Address H
  OUTBUFFER[5] = 0x00;
  OUTBUFFER[6] = 0x00;
  OUTBUFFER[7] = Servo_ID;	// ID of servo
  OUTBUFFER[8] = 0x00;
  OUTBUFFER[9] = 0x00;
  OUTBUFFER[10] = 0x00;
  OUTBUFFER[11] = 0x00;
  OUTBUFFER[12] = 0x4C;
  OUTBUFFER[13] = 0x1D;
  OUTBUFFER[14] = 0x00;
  OUTBUFFER[15] = 0x00;
  OUTBUFFER[16] = 0x00;
  OUTBUFFER[17] = 0x00;
  OUTBUFFER[18] = 0x00;
  OUTBUFFER[19] = 0x00;
  OUTBUFFER[20] = 0xFF;
  OUTBUFFER[21] = 0xFF;
  OUTBUFFER[22] = 0x01;
  OUTBUFFER[23] = 0xFF;
  OUTBUFFER[24] = 0xFF;
  OUTBUFFER[25] = 0x01;
  OUTBUFFER[26] = kondo_checksum(OUTBUFFER, 26); // Checksum

  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  if(debug == 1)
  {
    printf("DEBUG Single servo ID address: ");
    for(int i = 0; i < 27; i++)
    {
      printf("%X ", OUTBUFFER[i]);
    }
    printf("\n");
  }

  return returnVal;
}

/*
 * Function kondo_init_servo_ID - This function tells the RCB4 board, which 
 * of the servos in SERVOS[] are connected and to be initialised. 
 *
 * Active (selected) servos are denoted by a '1' in SERVOS[]
 *
 * ToDo: Change SERVOS[] to a bool; add SERVOS[] to KondoRef struct
 */
int kondo_init_servo_ID()
{
  //Activating hte selected servos
  OUTBUFFER[0] = 0x4D;			// Number of Bytes being sent
  OUTBUFFER[1] = 0x00;
  OUTBUFFER[2] = 0x02;
  OUTBUFFER[3] = 0x44;
  OUTBUFFER[4] = 0x00;
  OUTBUFFER[5] = 0x00;

  int j = 6; 		// offset from OUTBUFFER index for start of SERVO info

  unsigned int servo_address = SERVO_START;
  unsigned int ADDH;
  unsigned int ADDL;

  // Activate only the selected servos in SERVOS[]
  for (int i = 0; i < 35; i++)
  {
    if (SERVOS[i] == 1)
    {
      ADDH = (unsigned int) servo_address >> 8;
      ADDL = (unsigned int) servo_address & 0x00FF;

      OUTBUFFER[j]   = ADDL;
      OUTBUFFER[j+1] = ADDH;
    }
    else
    {
      OUTBUFFER[j]   = 0xFF;
      OUTBUFFER[j+1] = 0xFF;
    }
    j += 2;
    servo_address += SERVO_SIZE;
  }

  OUTBUFFER[76] = kondo_checksum(OUTBUFFER, 76);
  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  printf("DEBUG2: ");
  for(int i = 0; i < 77; i++)
  {
    printf("%X ", OUTBUFFER[i]);
  }

  return returnVal;
}

/*
 * Function kondo_servo_free - Puts the selected servos in FREE mode
 * i.e. they are able to be moved freely.
 *
 * ToDo: Look at modifying or cerating a function for individual servos
 */
int kondo_servo_free()
{
  OUTBUFFER[0] = 0x4F;
  OUTBUFFER[1] = 0x10;
  OUTBUFFER[2] = 0xFF;
  OUTBUFFER[3] = 0xFF;
  OUTBUFFER[4] = 0xFF;
  OUTBUFFER[5] = 0xFF;
  OUTBUFFER[6] = 0x07;
  OUTBUFFER[7] = 0x01;

  unsigned int FPosH = (unsigned int) FREE_POS >> 8;
  unsigned int FPosL = (unsigned int) FREE_POS & 0x00FF;
  int j = 8;		// offset from OUTBUFFER index for the start of SERVO info

  // Set all 35 servos to FREE
  for(int i = 0; i < MAX_SERVOS; i++)
  {
    OUTBUFFER[j]   = FPosL;
    OUTBUFFER[j+1] = FPosH;
    j += 2;
  }

  OUTBUFFER[78] = kondo_checksum(OUTBUFFER, 78);
  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  return returnVal;
}

/* Function kondo_servo_on - This powers on any servos selected
 * by the function kondo_init_servo_ID.
 */

int kondo_servo_on()
{
  OUTBUFFER[0] = 0x09;
  OUTBUFFER[1] = 0x00;
  OUTBUFFER[2] = 0x02;
  OUTBUFFER[3] = 0x00;
  OUTBUFFER[4] = 0x00;
  OUTBUFFER[5] = 0x00;
  OUTBUFFER[6] = 0x11;
  OUTBUFFER[7] = 0x00;
  OUTBUFFER[8] = 0x1C;	// Hardcoded checksum (as this doesn't change)

  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  return returnVal;
}

/*
 * Function kondo_set_servo_ID_pos - This function sets the position of
 * the servo indexed by IDX to the position set by aPOS.
 *
 * @param int IDX - the index of the servo, i.e. the servos own ID
 * @param int SIO - the serial IO channel the servo sits on (i.e. 0 or 1)
 * @param unsigned int aPos - the position to move the servo to (i.e. 7500)
 */
int kondo_set_servo_ID_pos(int IDX, int SIO, unsigned int aPos)
{
  IDX = IDX * 2; // this is the position in the array of where the servo sits.
  if (SIO == 1)
  {
    IDX++;
  }

  unsigned int POSH = (unsigned int) aPos >> 8;
  unsigned int POSL = (unsigned int) aPos & 0x00FF;

  OUTBUFFER[0] = 0x0A;
  OUTBUFFER[1] = 0x00;
  OUTBUFFER[2] = 0x12;
  OUTBUFFER[3] = 0x06;
  OUTBUFFER[4] = IDX;
  OUTBUFFER[5] = 0x00;
  OUTBUFFER[6] = POSL;
  OUTBUFFER[7] = POSH;
  OUTBUFFER[8] = 0x00;
  OUTBUFFER[9] = kondo_checksum(OUTBUFFER, 9);

  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 4);

  return returnVal;
}

/* Function kondo_read_analog_values reads the pins for analog input
 * Battery, AD1, AD2, AD3, etc
 * Analog number 0 is the battery voltage
 * Analog numbers 1-11 are the PINS.
 */
int kondo_read_analog_values(int result, UINT num)
{
  int i;

  // Check port number range
  if (num < 0 || num > 10)
    printf("Error: Invalid analog port number.\n");

  // memory locations of the requested analog values
  int mem_h = RCB4_ADDR_AD_READ_BASE + (num * 2);
  int mem_l = RCB4_ADDR_AD_REF_BASE + (num * 2);
  int mem_h_val = 0, mem_l_val = 0;

  // Build command to read mem_h from RAM to COM
  OUTBUFFER[0] = 10;			// num of bytes
  OUTBUFFER[1] = RCB4_CMD_MOV;		// Move command
  OUTBUFFER[2] = RCB4_RAM_TO_COM;	// RAM to COM
  OUTBUFFER[3] = 0;			// dest addr L (0 for COM)
  OUTBUFFER[4] = 0;			// dest addr M (0 for COM)
  OUTBUFFER[5] = 0;			// dest addr H (0 for COM)
  OUTBUFFER[6] = (UCHAR) (mem_h);	// mem_h low byte
  OUTBUFFER[7] = (UCHAR) (mem_h >> 8);	// mem_h high byte
  OUTBUFFER[8] = 2;			// bytes to move
  OUTBUFFER[9] = kondo_checksum(OUTBUFFER, 9);

  // send 10 bytes, expect 5 bytes
  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 5);

  // Decode mem_h value (which may be negative)
  mem_h_val = ((mem_h_val | INBUFFER[3]) << 8) | INBUFFER[2];
  if(mem_h_val > 0x8000)
    mem_h_val = -(~mem_h_val & 0x7fff) -1;

  // Build command to read mem_l from RAM to COM
  OUTBUFFER[0] = 10;                    // num of bytes
  OUTBUFFER[1] = RCB4_CMD_MOV;          // Move command
  OUTBUFFER[2] = RCB4_RAM_TO_COM;       // RAM to COM
  OUTBUFFER[3] = 0;                     // dest addr L (0 for COM)
  OUTBUFFER[4] = 0;                     // dest addr M (0 for COM)
  OUTBUFFER[5] = 0;                     // dest addr H (0 for COM)
  OUTBUFFER[6] = (UCHAR) (mem_l);       // mem_h low byte
  OUTBUFFER[7] = (UCHAR) (mem_l >> 8);  // mem_h high byte
  OUTBUFFER[8] = 2;                     // bytes to move
  OUTBUFFER[9] = kondo_checksum(OUTBUFFER, 9);

  // send 10 bytes, expect 5 bytes
  returnVal = kondo_send(&ki);
  returnVal = kondo_read(&ki, 5);

  // Deconde mem_l vlaue (which might be negative)
  mem_l_val = ((mem_l_val | INBUFFER[3]) << 8) | INBUFFER[2];
  if(mem_l_val > 0x8000)
    mem_l_val = -(~mem_l_val & 0x7fff) -1;

  result = mem_h_val + mem_l_val;

  return 0;
}


int main ( void )
{
  int ret = kondo_create_instance(&ki);
  if (ret < 0)
  {
    printf("Some Error\n");
    exit(-1);
  }

  if(kondo_ack() == -2)
  {
    printf("Error: Board not connected.\n");
    exit(-1);
  }
  else
  {
    printf("Sucess! ACK!\n");
  }

  // Test light
  kondo_led(true);
  printf("Press ANY key!");
  getchar();
  kondo_led(false);

  kondo_servo_ID_address(4, false);
  printf("Press ANY key!");
  getchar();


  SERVOS[8] = 0x01;     // location within the array i.e. IDX * 2 + SIO

  kondo_init_servo_ID();  // activate servo 4

  printf("Press ANY key!");
  getchar();


  kondo_servo_free();
  kondo_servo_on();

  printf("Press ANY key!");
  getchar();

  kondo_set_servo_ID_pos(4, 0, 7000);

  printf("Press ANY key!");
  getchar();

  kondo_close_instance(&ki);

  return 0;
}
