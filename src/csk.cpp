#include <arduino.h>
#include <stdint.h>
#include <SoftwareSerial.h>
#include "ov528/ov528.h"

// ov528 protocol.
#define OV528_BAUD 115200 // 57600 115200
#define OV528_PKT_SZ     512
#define OV528_SIZE       OV528_SIZE_QVGA
#define OV528_ADDR       (0<<5)

#define ACK_TIMEOUT 500
#define RETRY_LIMIT 200
#define TIMEOUT 500

SoftwareSerial sserial(11,13);
bool debug = false;
bool errorDetection = false;


//String header = "ffd8ffdb008400140e0f120f0d14121012171514181e32211e1c1c1e3d2c2e243249404c4b47404645505a736250556d5645466488656d777b8182814e608d978c7d96737e817c011517171e1a1e3b21213b7c5346537c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7c7cffc0001108007800a003012100021101031101ffdd0004000affc401a20000010501010101010100000000000000000102030405060708090a0b100002010303020403050504040000017d01020300041105122131410613516107227114328191a1082342b1c11552d1f02433627282090a161718191a25262728292a3435363738393a434445464748494a535455565758595a636465666768696a737475767778797a838485868788898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa0100030101010101010101010000000000000102030405060708090a0b1100020102040403040705040400010277000102031104052131061241510761711322328108144291a1b1c109233352f0156272d10a162434e125f11718191a262728292a35363738393a434445464748494a535455565758595a636465666768696a7374757677";



void debugCSK(uint8_t resp[],boolean debug)
{

	  if(debug)
	  {
		//Serial.println(readt);
		Serial.print(String(resp[0],HEX));
		Serial.print(",");
		Serial.print(String(resp[1],HEX));
		Serial.print(",");
		Serial.print(String(resp[2],HEX));
		Serial.print(",");
		Serial.print(String(resp[3],HEX));
		Serial.print(",");
		Serial.print(String(resp[4],HEX));
		Serial.print(",");
		Serial.print(String(resp[5],HEX));
		Serial.println();
	  }
}



void writeBytes(uint8_t buf[], uint16_t len)
{

  sserial.write(buf, len);

}

uint16_t readBytes(uint8_t buf[], uint16_t len, uint16_t timeout_ms) {
  uint16_t i;
  uint8_t subms = 0;
  for (i = 0; i < len; i++) {
    while (sserial.available() == 0) {
      delayMicroseconds(10);
      if (++subms >= 100) {
        if (timeout_ms == 0) {
          return i;
        }
        subms = 0;
        timeout_ms--;
      }
    }
    buf[i] = sserial.read();
  }
  return i;
}

uint8_t chkAck(uint8_t cmd)
{
  uint8_t resp[6];


  int readt = readBytes(resp, 6, TIMEOUT);

  debugCSK(resp,debug);

  if (readt != 6) {
    return 0;
  }
  if (resp[0] == 0xaa && resp[1] == (OV528_CMD_ACK | OV528_ADDR) && resp[2] == cmd && resp[4] == 0 && resp[5] == 0) {
    return 1;
  }
  return 0;
}

inline void recv_clear(void)
{
  sserial.flush();
}


void sendCmd(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
  uint8_t buf[6] = { 0xaa };
  buf[1] = cmd | OV528_ADDR;
  buf[2] = p1;
  buf[3] = p2;
  buf[4] = p3;
  buf[5] = p4;
  writeBytes(buf, 6);
}

inline uint8_t send_cmd_with_ack(uint8_t cmd, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  while (1) {
    recv_clear();
    sendCmd(cmd, p1, p2, p3, p4);
    if (chkAck(cmd)) {
      break;
    }
  }
  return 1;
}


void set_camera_reset() {


    sendCmd(0x08, 0x00, 0x00, 0x00, 0x00);
}

void set_camera_baudRate() {
  uint8_t resp[6];
  recv_clear();

    sendCmd(0x07, 0x00, 0x01, 0x00, 0x00);
}


void camera_sync() {
  uint8_t resp[6];
  recv_clear();
  while (1) {
    sendCmd(OV528_CMD_SYNC, 0, 0, 0, 0);
    if (!chkAck(OV528_CMD_SYNC)) {
      continue;
    }

    int readt = readBytes(resp, 6, TIMEOUT);

    debugCSK(resp,debug);

    if (readt != 6) continue;

    if (resp[0] == 0xaa &&
    	resp[1] == (OV528_CMD_SYNC | OV528_ADDR) &&
		resp[2] == 0 &&
		resp[3] == 0 &&
		resp[4] == 0 &&
		resp[5] == 0)

    	break;
  }

  sendCmd(OV528_CMD_ACK, OV528_CMD_SYNC, 0, 0, 0);
}


void camera_init(void)
{
	send_cmd_with_ack(0x01, 0x00, 0x03, 0x03, 0x03);
}


void camera_snapshot(void)
{
  send_cmd_with_ack(0x06, 0x08, OV528_PKT_SZ & 0xff, (OV528_PKT_SZ >> 8) & 0xff , 0);
  //send_cmd_with_ack(0x05, 0x00, 0x00,0x00, 0x00);
}


uint8_t camera_get_data()
{
  uint32_t data_size;
  uint8_t resp[6];
  byte buf[OV528_PKT_SZ];
  uint32_t photo_size = 0;


  while (1)
  {

    send_cmd_with_ack(0x04, 0x01, 0, 0, 0);

    if (readBytes(resp, 6, 1000) != 6)
    {
      continue;
    }

    debugCSK(resp,debug);


    if (resp[0] == 0xaa && resp[1] == (OV528_CMD_DATA | OV528_ADDR) && resp[2] == 0x01)
    {
      data_size = (resp[3]) | (resp[4] << 8) | ((uint32_t)resp[5] << 8);
      break;
    }
  }

  uint8_t num_packet = (data_size + (OV528_PKT_SZ - 6 - 1)) / (OV528_PKT_SZ - 6);



  if(false)
  {
	  Serial.println();
	  Serial.print("packet number: ");
	  Serial.println(num_packet);
  }
  //size_fun(ctx, data_size);


  //Serial.print(header);

  for (uint16_t i = 1; i < num_packet; i++)
  {

    uint8_t retry_cnt = 0;

retry:

    //recv_clear();
    sendCmd(OV528_CMD_ACK, 0, 0,  i & 0xff, (i >> 8) & 0xff);



    // recv data : 0xaa, OV528_CMD_DATA, len16, data..., sum?
    uint16_t len = readBytes(buf, OV528_PKT_SZ, 200);

    if(debug)
    {
		Serial.println();
		Serial.print("Packet number: ");
		Serial.println(i+1);
		Serial.print("Length: ");
		Serial.println(len);

    }

    //Make sure bytes less than 0xe are written as 0x0e
    //for (uint16_t i = 4; i < len-1; i++)

	if(!errorDetection)
	{
		for (uint16_t j = 4; j < len-2; j++)
		{
			if(buf[j]<16)
			{
				//Serial.print("0x0"+ String(buf[j],HEX) + " ");
				Serial.print("0" + String(buf[j],HEX) + "");
			}
			else
			{
				//Serial.print("0x"+ String(buf[j],HEX) + " ");
				Serial.print("" + String(buf[j],HEX) + "");
			}

		}

		if(i == num_packet-1)
		{
			for (int i = len; i < 512; i++)
			{

				Serial.print("00");

			}

			Serial.println();

		}
	}



	if(errorDetection)
	{
		//Checksum
		uint8_t sum = 0;
		for (uint16_t y = 0; y < len - 2; y++)
		{
		  sum += buf[y];
		}

		//Serial.println();
		//Serial.print("checksum: ");
		//Serial.println(sum);

		if (sum != buf[len - 2])
		{
		  if (++retry_cnt < RETRY_LIMIT)
		  {
			delay(100);
			goto retry;
		  }
		  else
		  {
			sendCmd(OV528_CMD_ACK, 0, 0, 0xf0, 0xf0);
			return 0;
		  }
		}
		else
		{

			for (uint16_t j = 4; j < len-2; j++)
			{

				if(buf[j]<16)
				{
					//Serial.print("0x0"+ String(buf[j],HEX) + " ");
					Serial.print("0" + String(buf[j],HEX) + "");
				}
				else
				{
					//Serial.print("0x"+ String(buf[j],HEX) + " ");
					Serial.print("" + String(buf[j],HEX) + "");
				}

			}

			photo_size += (len-6);
		}
	}

	photo_size += (len-6);

  }


  //Serial.print("Photo size: ");
  //Serial.println(photo_size);

  sendCmd(OV528_CMD_ACK, 0, 0, 0xf0, 0xf0);
  return 1;
}




void setup(void)
{
	//Baude rate set to 115200
	Serial.begin(115200);
	sserial.begin(115200);

	if(debug)
		Serial.println("****reset****");
	set_camera_reset();

	if(debug)
		Serial.println("****sync****");
	camera_sync();

	if(debug)
		Serial.println("****baudrate****");
	set_camera_baudRate();

	if(debug)
		Serial.println("****init****");
	camera_init();

	if(debug)
		Serial.println("****snapshot****");
	camera_snapshot();

}


int incomingByte = 0;

void loop(void)
{

	if(debug)
		Serial.println("****get_data****");
	camera_get_data();


    while (Serial.available() > 0)
    {

            incomingByte =  Serial.read();

            Serial.print("");
            Serial.print(String(incomingByte,HEX));
            Serial.println("");

    }
}

