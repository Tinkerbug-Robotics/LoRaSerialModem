// Used for hardware serial on GPIO pins
#include <Arduino.h>

// Radio library for working with radio
#include <RadioLib.h>

// Max line length
#define MAX_PACKET_LENGTH 250

// Max number of packets in a burst
#define MAX_NUM_PACKETS 4

// Serial connection to GPS receiver on GPIO pins
UART Serial2(0, 1, 0, 0);

// Indicates that data is available to add
bool data_to_add = false;

// A character array for one GPS message
typedef struct char_array {char x[MAX_PACKET_LENGTH];} char_array;

// Invalid array, returned when out of bound
char_array default_invalid_array;

typedef struct gps_data_and_length
{
    char_array gps_data;
    uint8_t packet_length;
} gps_data_and_length;

// Structure to contain stored GPS messages
typedef struct circular_buffer
{
    char_array gps_data[MAX_NUM_PACKETS];
    uint8_t packet_length[MAX_NUM_PACKETS];
    uint8_t head;
    uint8_t tail;
} circular_buffer;

circular_buffer cb;

// SX1276 has the following connections:
// NSS pin:   17
// DIO0 pin:  20
// RESET pin: 21
// DIO1 pin:  22
SX1278 radio = new Module(17, 20, 21, 22);

// Routine to initialize the circular buffer
void cb_init(circular_buffer *cb)
{
    Serial.println("init buffer");
    cb->head = 0;
    cb->tail = 0;
}

void setup()
{

    // USB serial
    Serial.begin(115200);

    // TODO: Remove when not running with computer
    //while (!Serial){};

    // Initialize buffer and invalid array
    cb_init(&cb);
    for (uint8_t i=0; i<MAX_PACKET_LENGTH; i++)
    {
        default_invalid_array.x[i]=0;
    }

    Serial.println("Starting Pico Radio ...");
  
    // Start serial connection to GPS receiver
    Serial2.begin(57600);

    // Initialize SX1276 with default settings
    Serial.print("SX1276 Initializing ... ");
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

    // Print success or failure
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("Success!");
    }
    else
    {
        Serial.print("failed, code ");
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

// Main loop
void loop()
{
    // Read serial buffer and push LoRa sized (250 byte) packets to cicular buffer
    readSerialBuffer();
  
    // If there is data to transmit and the serial data from GPS is all received, then transmit data over radio
    if (cb.head != cb.tail)
    {
        transmit_data();
    }
}

// Read serial buffer and pack data into LoRa sized (250 byte) packets
void readSerialBuffer ()
{

    // Packet data to be filled and pushed to buffer
    static char_array packet;

    // Position through current packet
    static uint8_t input_pos = 0;

    // Serial character
    byte in_byte;

    // If serial data is still available
    while (Serial2.available ())
    {
        // Fill the packet with characters
        if (input_pos < MAX_PACKET_LENGTH)
        {
            // Read data from serial
            in_byte = Serial2.read();

            packet.x[input_pos] = in_byte;
            data_to_add = true;
            //Serial.println(in_byte, HEX);
            input_pos++;

        }
        // If the packet is full, then add it to the circular buffer
        else if (input_pos == MAX_PACKET_LENGTH)
        {
 
            // Push packet and packet length into buffer
            cb_push_front(&cb, packet, input_pos);
            
            Serial.print(millis());Serial.print(" Saving length: ");Serial.println(input_pos);
            
            // Return length counter to zero
            input_pos = 0;

            data_to_add = false;
        }   

    }

}

// Transmit available data over radio
void transmit_data ()
{

    // Pointer to first element of current row of burst array
    gps_data_and_length single_packet = cb_pop_back(&cb);
    
    unsigned int start_time = millis();
    int state = radio.transmit((uint8_t *)single_packet.gps_data.x, single_packet.packet_length);
    unsigned int end_time = millis();

    Serial.print(millis());Serial.print(" Sending - Length: ");Serial.print(single_packet.packet_length);
    Serial.print(" Transmit time: ");Serial.print(end_time - start_time);
    Serial.print(" Datarate: ");Serial.println(radio.getDataRate());

    // If an error has occured, print it
    if (state != 0)
        printRadioError(state);
    
}

// Push new data onto cicular buffer
void cb_push_front(circular_buffer *cb, char_array item, uint8_t packet_length)
{
    int next_head = cb->head + 1;
    if(next_head >= MAX_NUM_PACKETS)
        next_head = 0;
    if(next_head == cb->tail)
    {
        Serial.println("buffer overflow");
        return;
    }
    cb->gps_data[cb->head] = item;
    cb->packet_length[cb->head] = packet_length;
    cb->head = next_head;
}

// Pop data off circular buffer for use
gps_data_and_length cb_pop_back(circular_buffer *cb)
{
    gps_data_and_length gp;
    
    if(cb->tail == cb->head)
    {
        Serial.println("buffer underflow");
        gp.packet_length = 0;
        gp.gps_data = default_invalid_array;
        return gp;
    }
    gp.packet_length = cb->packet_length[cb->tail];
    gp.gps_data = cb->gps_data[cb->tail];
    cb->tail++;
    if(cb->tail >= MAX_NUM_PACKETS)
        cb->tail = 0;
    return gp;
}

void printRadioError(int state)
{
    // Packet too long
    if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
    {
        Serial.println(F("Packet is too long"));

    }
    // Radio timed out
    else if (state == RADIOLIB_ERR_TX_TIMEOUT)
    {
        Serial.println(F("Radio timeout"));

    }
    // Other error code
    else
    {
        Serial.print(F("failed, code is "));
        Serial.println(state);
    }
}
