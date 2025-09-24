#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <RadioLib.h> // Include the RadioLib library

// Define LoRa pins for Heltec Wireless Stick V3 (ESP32-S3)
// These are common pin definitions, but ALWAYS VERIFY with your board's schematic.
#define LORA_CS 8
#define LORA_RST 4
#define LORA_IRQ 47 // GPIO47 is often used for DIO1/IRQ on SX126x
#define LORA_BUSY 48 // GPIO48 is often used for BUSY on SX126x

// If you are using a board with an antenna switch (like some Heltec boards)
// you might need to define these and pass them to the Module constructor.
// For example:
// #define LORA_RXEN 21 // Example RX Enable pin (if applicable)
// #define LORA_TXEN 14 // Example TX Enable pin (if applicable)

// Create a new instance of the SX1262 radio
// Module(CS, DIO1, RST, BUSY) for SX126x
// If you have antenna switching pins, you'd add them here:
// SX1262 radio = new Module(&SPI, LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY, LORA_RXEN, LORA_TXEN);
SX1262 radio = new Module(LORA_CS, LORA_IRQ, LORA_RST, LORA_BUSY);

#define BAND 915.0 // Frequency in MHz
#define debug true // Set to true for serial debug messages
// #define NO_DISPLAY // This is no longer strictly necessary as we are not using Heltec display functions

float tensao = 0.0; // This variable is not used in the provided snippet

const int analogPin_36 = 12; // Example: ADC1_CH0 on ESP32-S3. Verify with schematic.
const int analogPin_39 = 15; // Example: ADC1_CH1 on ESP32-S3. Verify with schematic.


// Tensão de referência do ADC (3.3V para o ESP32-S3)
const float referenceVoltage = 3.3;

// Resolução do ADC (12 bits por padrão, variando de 0 a 4095)
const int adcResolution = 4095;

int analogValue_ch36;
int analogValue_ch39;
float voltage_1;
float voltage_2;

unsigned int counter = 0;
String rssi = "RSSI --"; // Not used in this code, but kept for context
String packSize = "--";   // Not used in this code, but kept for context
String packet;            // Not used in this code, but kept for context

char msgMQTT[500]; // Increased buffer size just in case, consider StaticJsonDocument size

unsigned long tempo1 = 0; // For non-blocking delay


void adc_esp() {
    // Read analog values directly from the specified pins
    analogValue_ch36 = analogRead(analogPin_36);
    analogValue_ch39 = analogRead(analogPin_39);

    // Convert the read values to voltage
    voltage_1 = analogValue_ch36 * (referenceVoltage / adcResolution);
    voltage_2 = analogValue_ch39 * (referenceVoltage / adcResolution);

   if (debug) {
        // Imprime o valor lido e a tensão convertida no monitor serial
        Serial.print("Valor analógico lido (Pin "); Serial.print(analogPin_36); Serial.print("): ");
        Serial.print(analogValue_ch36);
        Serial.print(" -> Tensão 1: ");
        Serial.print(voltage_1, 2); // Format to 2 decimal places
        Serial.println(" V");

        Serial.print("Valor analógico lido (Pin "); Serial.print(analogPin_39); Serial.print("): ");
        Serial.print(analogValue_ch39);
        Serial.print(" -> Tensão 2: ");
        Serial.print(voltage_2, 2); // Format to 2 decimal places
        Serial.println(" V");
    }
}

void enviarPacote() {
    /*
     * envio dos pacotes via rádio LoRa
     */

    StaticJsonDocument<500> doc; // Adjust size if your JSON grows larger
    doc["topico"] = "deviceSender";
    doc["id"] = "deviceSender";
    doc["nivel"] = counter;
    doc["tensao_1"] = voltage_1;
    doc["tensao_2"] = voltage_2;
    doc["corrente"] = int(random(0, 100)); // Consider replacing with actual sensor reading

    serializeJson(doc, msgMQTT);

    // Set LoRa transmit power (e.g., 14 dBm, common for SX1262)
    // Check RadioLib documentation for valid power ranges for your module (e.g., -9 to 22 dBm for SX126x)
    int state = radio.setTxPower(14);
    if (state != RADIOLIB_ERR_NONE) {
        if (debug) Serial.printf("Failed to set Tx power, code: %d\n", state);
    }

    // Transmit the LoRa packet
    state = radio.startTransmit(msgMQTT); // startTransmit takes a string or byte array
    if (state == RADIOLIB_ERR_NONE) {
        if (debug) Serial.print("LoRa Packet Started: ");
    } else {
        if (debug) Serial.printf("Failed to start transmission, code: %d\n", state);
    }

    state = radio.finishTransmit(); // finishTransmit sends the packet
    if (state == RADIOLIB_ERR_NONE) {
        if (debug) Serial.println(msgMQTT);
    } else {
        if (debug) Serial.printf("Failed to finish transmission, code: %d\n", state);
    }
    counter++;
}


void setup() {
    Serial.begin(115200);
    Serial.println("Serial initial done");
    delay(100);
    Serial.println("If you see this, basic serial works.");

    if (debug) {
        Serial.println("Iniciando RadioLib LoRa...");
    }

    // Initialize LoRa module with RadioLib
    // Parameters for begin(): frequency, bandwidth, spreading factor, coding rate, output power, preamble length, gain
    // Common values for LoRa:
    // Bandwidth: 125.0 kHz
    // Spreading Factor: 9 (SF7-SF12 common)
    // Coding Rate: 7 (CR5-CR8 common)
    // Output Power: 14 dBm
    // Preamble Length: 8 symbols
    // Gain: 0 (automatic gain control)
    int state = radio.begin(BAND, 125.0, 9, 7, 14, 8, 0);

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("RadioLib LoRa successful!");
    } else {
        Serial.printf("Starting LoRa failed! Code: %d. Halting.\n", state);
        while (true); // Halt if LoRa fails
    }
}


void loop() {
    // server.handleClient();  // If you have a web server, uncomment and implement this

    if (millis() - tempo1 >= 5000) {  // Send packet every 5 seconds (adjust as needed)
        tempo1 = millis();
        // The 'LED' pin might be different on V3.
        // Heltec boards often have a user LED. Check your V3 schematic for the LED pin.
        // For V3, it's often GPIO35 or GPIO38.
        // If 'LED' is undefined or incorrect, this line might cause issues.
        // You might need to define #define LED <your_V3_LED_GPIO_pin>
        // Or simply remove this line if LED blinking is not critical.
        // digitalWrite(LED, not(digitalRead(LED))); // Temporarily comment out if LED pin is unknown
        adc_esp();  // Read ADC values first
        enviarPacote(); // Then send the packet with updated ADC values
    }
    // No delay here, as it's called in loop and would block server/LoRa operations
}