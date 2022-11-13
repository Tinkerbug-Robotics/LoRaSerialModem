// Used for hardware serial on GPIO pins
#include <Arduino.h>

// Radio library for working with radio
#include <RadioLib.h>

// Max line length
#define PACKET_LENGTH 250

// Max message length
#define MAX_MSG_LENGTH 2500

bool data_available = false;
bool last_char_start_of_msg = false;

char rtk_data[PACKET_LENGTH];

// Serial connection to GPS receiver's RXD2 pin that receives correction data
UART Serial2(0, 1, 0, 0);

// SX1276 has the following connections:
// NSS pin:   17
// DIO0 pin:  20
// RESET pin: 21
// DIO1 pin:  22
SX1278 radio = new Module(17, 20, 21, 22);

void setup()
{

    // USB serial
    Serial.begin(115200);

    // TODO: Remove when not running with computer
    while (!Serial){};

    Serial.println("Starting Pico Radio ...");
  
    // Start serial connection to GPS receiver
    Serial2.begin(57600);

    // Initialize SX1276 with default settings
    Serial.print(F("[SX1276] Initializing ... "));
    int state = radio.begin();
    
    // Frequency (kHz)
    // Bandwidth (kHz)
    // Spreading factor
    // Coding rate
    // Sync word
    // Output Power
    radio.setFrequency(915.0);
    radio.setBandwidth(500.0);
    radio.setSpreadingFactor(9);
    radio.setCodingRate(7);
    radio.setOutputPower(10);
    
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }
    // Overcurrent protection, Upto 120 mA
    radio.setCurrentLimit(60);
    // LoRa CRC
    radio.setCRC(true);
    // LoRa header mode
    radio.explicitHeader();
  
    Serial.println("Setup complete");

}

unsigned int msg_indx = 0;
static char rtcm_msg[MAX_MSG_LENGTH];
char last_byte = 0;
unsigned int msg_length = 99999;
bool in_message = false;
  
// Main loop
void loop()
{
    // Receive LoRa data from base station
    uint8_t state = radio.receive((uint8_t *)rtk_data, PACKET_LENGTH);
    printRadioError(state);

    // If a message is received
    if (state == RADIOLIB_ERR_NONE)
    {
        // Loop through received message
        for(uint8_t i=0;i<PACKET_LENGTH;i++)
        {

            // Read the next character in the message
            char in_byte = rtk_data[i];
            Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
            Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
            Serial.print("i: ");Serial.print(i);Serial.print(" ");Serial.println(in_byte, HEX);

            // Look for the start of a message
            if (!in_message && last_byte == 0xD3 && (in_byte & 0xFC) == 0x00)
            {
    
                rtcm_msg[0] = last_byte;
                rtcm_msg[1] = in_byte;
                rtcm_msg[2] = rtk_data[i+1];
                Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
                Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
                Serial.print("i: ");Serial.print(i+1);Serial.print(" ");Serial.println(rtk_data[2], HEX);
                rtcm_msg[3] = rtk_data[i+2];
                Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
                Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
                Serial.print("i: ");Serial.print(i+2);Serial.print(" ");Serial.println(rtk_data[3], HEX);
                rtcm_msg[4] = rtk_data[i+3];
                Serial.print("msg_length: ");Serial.print(msg_length);Serial.print(" ");
                Serial.print("msg_indx: ");Serial.print(msg_indx);Serial.print(" ");
                Serial.print("i: ");Serial.print(i+3);Serial.print(" ");Serial.println(rtk_data[4], HEX);

                msg_length = (((uint32_t)rtcm_msg[1] & 3) << 8) + ((uint32_t)rtcm_msg[2] << 0) + 6;
                unsigned int msg_num = ((uint32_t)rtcm_msg[3] << 4) + ((uint32_t)rtcm_msg[4] >> 4);

                Serial.print(millis());Serial.print(" Message Length: "); Serial.print(msg_length);
                Serial.print(" Message Number: "); Serial.println(msg_num);

                msg_indx = 5;
                i = i+3;

                in_message = true;
            }

            // End of a message (inside a packet)
            else if (msg_indx == msg_length-1)
            {

                // Add last character to RTCM message
                rtcm_msg[msg_indx]=in_byte;
            
                // Send message to GPS receiver
                Serial2.write(rtcm_msg, msg_length);

                Serial.print(Crc24Quick(0x000000, msg_length, rtcm_msg));

                Serial.println(" Sending message to GPS");

                // Done with message
                in_message = false;

                msg_length = 0;

                msg_indx = 0;

            }

            // Reading a message
            else if (in_message)
            {
                
                // Add last character to RTCM message
                rtcm_msg[msg_indx]=in_byte;

                msg_indx++;
            }
            
            last_byte = in_byte;
        }
    }
}

uint32_t Crc24Quick(uint32_t Crc, uint32_t Size, char *Buffer)
{
  static const uint32_t crctab[] = {
    0x00000000,0x01864CFB,0x038AD50D,0x020C99F6,0x0793E6E1,0x0615AA1A,0x041933EC,0x059F7F17,
    0x0FA18139,0x0E27CDC2,0x0C2B5434,0x0DAD18CF,0x083267D8,0x09B42B23,0x0BB8B2D5,0x0A3EFE2E };

  while(Size--)
  {
    Crc ^= (uint32_t)*Buffer++ << 16;
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
  }

  return(Crc & 0xFFFFFF);
}

void printRadioError(int state)
{
  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    data_available = true;

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("RSSI:\t\t\t"));
    Serial.print(radio.getRSSI());
    Serial.print(F(" dBm "));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("SNR:\t\t\t"));
    Serial.print(radio.getSNR());
    Serial.print(F(" dB "));

    // print frequency error
    // of the last received packet
    Serial.print(F("Frequency error:\t"));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    //Serial.print(F("failed, code "));
    //Serial.println(state);

  }
}
