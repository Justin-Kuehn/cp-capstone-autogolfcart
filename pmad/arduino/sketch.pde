/*

  Power Management Arduino Sketch
  Copyright 2010 Andrew Harris

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/*

  This sketch provides access to the A/D values on channels 0, 1, 2, and 3
  as well as the ability to control the digital outputs 4, 5, 6, and 7.

  The UART protocol used is called ROBIN, and it's described here:
     http://www.bdmicro.com/code/robin/

  This sketch uses a simplified version of the protocol -- <flags> is
  always zero.

  So to send something to this sketch over the UART, one would send
(binary data):
  0xaa 0x99 0x2 0x0 0x0 <len> <opcode> [argument1] [argument2] <cksum>
  Note that the destination address is #defined below and it's
currently set to 2.

  The following opcodes are supported currently:
     2   -> returns a status packet containing A/D values, digital
values, and a command count
     3   -> clears the command counter
    0xff -> returns the string "pmad", you can change this to be some
unique four letter ID
            for your device.  This is useful if you want to make sure
you're talking to the
            right device after you open the serial port.

  For more information, look at the source below.

*/

/* Start ROBIN stuff */

unsigned char test_index = 0;
unsigned char test_packet[9] = {
 0xaa, 0x99, 0x02, 0x01, 0x00, 0x02, 0x68, 0x69, 0xd6};

#define ROBIN_BAUDRATE 9600

typedef enum
{
 ROBIN_START,
 ROBIN_LEADIN,
 ROBIN_DEST,
 ROBIN_SRC,
 ROBIN_FLAGS,
 ROBIN_LEN,
 ROBIN_DATA,
 ROBIN_CKSUM
}
Robin_states_type;

#define ROBIN_DEST_ADDR 0x2

#define ROBIN_START_CHAR 0xaa
#define ROBIN_LEADIN_CHAR 0x99

#define ROBIN_MAX_DATA 32

int robin_state = ROBIN_START;
unsigned char robin_data[ROBIN_MAX_DATA];
int robin_data_index = 0;
int robin_our_addr = ROBIN_DEST_ADDR;
int robin_dest;
int robin_src;
int robin_flags;
int robin_data_len;
int robin_cksum;

void RobinInit()
{
 Serial.begin(ROBIN_BAUDRATE);
}

int ProcessRobinByte()
{
 int c;
 int i, cksum;

 if (robin_data_index >= ROBIN_MAX_DATA)
 {
   robin_state = ROBIN_START;
 }

 c = Serial.read();
 //c = test_packet[test_index];
 //test_index++;

 switch(robin_state)
 {
 case ROBIN_START:
   if (c == ROBIN_START_CHAR)
   {
     robin_state = ROBIN_LEADIN;
   }
   break;
 case ROBIN_LEADIN:
   if (c == ROBIN_LEADIN_CHAR)
   {
     robin_state = ROBIN_DEST;
   }
   else
   {
     robin_state = ROBIN_START;
   }
   break;
 case ROBIN_DEST:
   if (c == robin_our_addr || robin_our_addr == 0xff)
   {
     robin_dest = c;
     robin_state = ROBIN_SRC;
   }
   else
   {
     robin_state = ROBIN_START;
   }
   break;
 case ROBIN_SRC:
   robin_src = c;
   robin_state = ROBIN_FLAGS;
   break;
 case ROBIN_FLAGS:
   robin_flags = c;
   robin_state = ROBIN_LEN;
   break;
 case ROBIN_LEN:
   if (c < ROBIN_MAX_DATA)
   {
     robin_data_len = c;
     robin_data_index = 0;
     robin_state = ROBIN_DATA;
   }
   else
   {
     robin_state = ROBIN_START;
   }
   break;
 case ROBIN_DATA:
   robin_data[robin_data_index] = c;
   robin_data_index++;
   if (robin_data_index == robin_data_len)
   {
     robin_state = ROBIN_CKSUM;
   }
   break;
 case ROBIN_CKSUM:
   cksum = (robin_dest + robin_src + robin_flags + robin_data_len) & 0xff;
   for(i = 0; i < robin_data_len; i++)
   {
     cksum += robin_data[i];
   }
   cksum &= 0xff;
   robin_state = ROBIN_START;
   if (cksum == c)
   {
     return 1;
   }
   break;
 }
 return 0;
}

void SendRobinResponse(int data_len)
{
 int i, cksum;

 Serial.print(0xaa, BYTE);
 Serial.print(0x99, BYTE);
 Serial.print(robin_src, BYTE); // send the source as the new destination
 Serial.print(robin_dest, BYTE); // send us as the source
 Serial.print(robin_flags, BYTE); // respond with the same flags SUSP
 Serial.print(data_len, BYTE);

 cksum = (robin_src + robin_dest + robin_flags + data_len) & 0xff;
 for (i = 0; i < data_len; i++)
 {
   cksum += robin_data[i];
   Serial.print(robin_data[i], BYTE);
 }
 cksum &= 0xff;
 Serial.print(cksum, BYTE);
}

/* end ROBIN stuff */

int cmds_executed = 0;

void setup()
{
 RobinInit();

 /* start the outputs in HIGH because I'm using solid state relays
that consider LOW to be ON. */
 /* and I want the system to start with everything off. */
 pinMode(4, OUTPUT);
 digitalWrite(4, LOW);

 pinMode(5, OUTPUT);
 digitalWrite(5, LOW);

 pinMode(6, OUTPUT);
 digitalWrite(6, LOW);

 pinMode(7, OUTPUT);
 digitalWrite(7, LOW);

 pinMode(13, OUTPUT);
}

void loop()
{
 if (Serial.available() > 0)
 {
   if (ProcessRobinByte() > 0)
   {
     /* ProcessRobinByte() returns > 0 when a complete command is received */
     ProcessCommand(robin_data);
   }
 }
}

void GetADCValue(int pin, unsigned int *result)
{
 int i;
 unsigned int v;

 /* read the analog value 16 times and average the result */
 v = 0;
 for (i = 0; i < 16; i++)
 {
   v += analogRead(pin);
 }
 *result = v >> 4;
}

int ProcessCommand(unsigned char *cmd_buf)
{
 int i;
 unsigned int temp;

 cmds_executed++;

 /* first byte of command is opcode */
 /* second byte of command is what pin to write to */
 /* last byte of command is either 1 or 0 indicating high or low */

 switch(cmd_buf[0])
 {
 case 2:
   GenerateStatusPacket(robin_data);
   break;
 case 3:
   cmds_executed = 0;
   if (cmd_buf == robin_data)
   {
     SendRobinResponse(1);
   }
 case 4:
   /* digital pin set */
   //digitalWrite(cmd_buf[1], cmd_buf[2] == 0 ? LOW : HIGH);
   analogWrite(5, (255 * (cmd_buf[2]/100)));
   if (cmd_buf == robin_data)
   {
     SendRobinResponse(1);
   }
   break;
 case 0xff:
   cmd_buf[1] = 0;
   cmd_buf[2] = 'p';
   cmd_buf[3] = 'm';
   cmd_buf[4] = 'a';
   cmd_buf[5] = 'd';
   SendRobinResponse(6);
   break;
 default:
   break;
 }
}

void GenerateStatusPacket(unsigned char *cmd_buf)
{
 int i;
 unsigned int temp;

 cmd_buf[1] = 0;

 /* analog channels read */
 for(i = 0; i < 4; i++)
 {
   GetADCValue(i, &temp);
   cmd_buf[i * 2 + 2] = (temp >> 8) & 0xff;
   cmd_buf[i * 2 + 3] = temp & 0xff;
 }

 /* digital pin read */
 cmd_buf[10] = 0;
 cmd_buf[11] = digitalRead(4);
 cmd_buf[12] = digitalRead(5);
 cmd_buf[13] = digitalRead(6);
 cmd_buf[14] = digitalRead(7);

 cmd_buf[15] = (cmds_executed >> 8) & 0xff;
 cmd_buf[16] = cmds_executed & 0xff;

 SendRobinResponse(17);
}

