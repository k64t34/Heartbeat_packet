/*

Demonstrates the usage of heartbeat packets between radio nodes. 

Radio    Arduino
CE    -> 7
CSN   -> 8 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> 2  (Hardware INT1)
VCC   -> No more than 3.6 volts
GND   -> GND

*/

#include <SPI.h>
#include <NRFLite.h>

const static uint8_t MY_PULSE = 10000; //Time interval (millisecond) through which the heartbeat packets should be sent from this node
const static uint8_t PULSE_FOR_ME = 1000; //Time interval (millisecond) through which the heartbeat packets should be sent to this node

unsigned long Next_MY_PULSE;
unsigned long Next_PULSE_FOR_ME;

const static uint8_t RADIO_ID = 0;
const static uint8_t DESTINATION_RADIO_ID = 0;
const static uint8_t PIN_RADIO_CE = 7;
const static uint8_t PIN_RADIO_CSN = 8;
const static uint8_t PIN_RADIO_IRQ = 2;

NRFLite _radio;
uint8_t _data;
volatile uint8_t _dataWasReceived; // Note usage of volatile for the global variable being changed in the radio interrupt.

void setup()
{
    Serial.begin(115200);

    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }

    attachInterrupt(digitalPinToInterrupt(PIN_RADIO_IRQ), radioInterrupt, FALLING);
	
    unsigned long time = millis();
    Next_MY_PULSE     = time + MY_PULSE;
    Next_PULSE_FOR_ME = time + PULSE_FOR_ME;
}

void loop()
{
	unsigned long time = millis();
	
	if (_dataWasReceived)
    {
        _dataWasReceived = false;

        // Use 'hasDataISR' rather than 'hasData' when using interrupts.
        while (_radio.hasDataISR())
        {
            uint8_t data;
            _radio.readData(&data);
            Serial.print("Received ");
            Serial.print(data);

            // Add an Ack data packet to the radio.  The next time we receive data,
            // this Ack packet will be sent back to the transmitting radio.
            uint8_t ackData = data;
            _radio.addAckData(&ackData, sizeof(ackData));

            Serial.print(", Added Ack ");
            Serial.println(ackData);
            Serial.flush(); // Serial uses interrupts so let's ensure printing is complete before processing another radio interrupt.
        }
		
		Next_PULSE_FOR_ME = time + PULSE_FOR_ME;
    }
	else 
  	{		
  	// Check for timeout incoming heartbeat packets
    if (time > Next_PULSE_FOR_ME)
      {
  		Serial.println("Alarm! No income heartbeat packets. Stop work");	
  		}
  	}
   
	// Is it time to send outcoming heartbeat packets?
	if ( time > Next_MY_PULSE)
    {
		if (_radio.send(DESTINATION_RADIO_ID, &_data, sizeof(_data), NRFLite::NO_ACK))
			Next_MY_PULSE = time + MY_PULSE;
    }	
}

void radioInterrupt()
{
    // Ask the radio what caused the interrupt.  This also resets the IRQ pin on the
    // radio so a new interrupt can be triggered.

    uint8_t txOk, txFail, rxReady;
    _radio.whatHappened(txOk, txFail, rxReady);

    // txOk = the radio successfully transmitted data.
    // txFail = the radio failed to transmit data.
    // rxReady = the radio received data.

    if (rxReady)
    {
        _dataWasReceived = true;
    }
}
