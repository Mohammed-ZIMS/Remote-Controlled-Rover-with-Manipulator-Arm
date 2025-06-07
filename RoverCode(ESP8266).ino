#include <SPI.h>
#include <RF24.h>

#define CE_PIN D4  // GPIO4
#define CSN_PIN D8 // GPIO15

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

void setup()
{
    Serial.begin(9600);
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.setChannel(100);
    radio.setDataRate(RF24_1MBPS);
    radio.openReadingPipe(0, address);
    radio.startListening(); // Set as receiver
}

void loop()
{
    if (radio.available())
    {
        char text[64] = {0}; // Increased buffer size to match transmitter
        radio.read(&text, sizeof(text));
        Serial.println(text); // Forward the received data to Arduino Uno
    }
    delay(1); // Small delay to prevent busy-waiting
}
