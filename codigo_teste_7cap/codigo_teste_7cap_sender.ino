#include <RadioLib.h>
#include <ArduinoJson.h>

// Módulo LoRa
SX1262 radio = new Module(8, 14, 12, 13);

int transmissionState = RADIOLIB_ERR_NONE;
volatile bool transmittedFlag = false;

// Pinos dos sensores
#define pinV1 7  // Tensão bateria principal (motor)
#define pinV2 6  // Tensão bateria auxiliar
#define pinV3 5  // Tensão painéis solares
#define pinI1 4  // Corrente motor (não usado, mas definido)
#define pinI2 3  // Corrente que entra no MPPT
#define pinI3 2  // Corrente que sai do MPPT

const float referenceVoltage = 3.3;
const int adcResolution = 4095;

float resolucao;

char msgMQTT[500];

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  transmittedFlag = true;
}

void setup() {
  Serial.begin(9600);

  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  radio.setFrequency(915);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setPacketSentAction(setFlag);

  Serial.print(F("[SX1262] Sending first packet ... "));
  Serial.println(transmissionState);
  transmissionState = radio.startTransmit("Hello World!");
  Serial.println(transmissionState);

  resolucao = referenceVoltage / adcResolution;
}

int count = 0;

void adc_esp() {
  // Leitura das tensões
  int rawV1 = analogRead(pinV1);
  int rawV2 = analogRead(pinV2);
  int rawV3 = analogRead(pinV3);

  // Leitura das correntes
  int rawI1 = analogRead(pinI1);
  int rawI2 = analogRead(pinI2);
  int rawI3 = analogRead(pinI3);

  // Conversão das tensões (baseado nos seus valores máximos)
  float tensao_motor      = rawV1 * resolucao * 16.48;   // até 54.4V
  float tensao_auxiliar   = rawV2 * resolucao * 3.64;    // até 12V
  float tensao_placas     = rawV3 * resolucao * 25.45;   // até 84V

  // Conversão das correntes (multiplica por 30.3 baseado no divisor)
  float corrente_motor    = rawI1 * resolucao * 30.3;    // sensor 0-100A (não usado antes)
  float corrente_entrada_mppt = rawI2 * resolucao * 30.3;  // até ~22A
  float corrente_saida_mppt   = rawI3 * resolucao * 30.3;  // até ~48A

  // Debug no Serial Monitor (opcional)
  Serial.println("Leituras brutas:");
  Serial.println("V1: " + String(rawV1) + " V2: " + String(rawV2) + " V3: " + String(rawV3));
  Serial.println("I1: " + String(rawI1) + " I2: " + String(rawI2) + " I3: " + String(rawI3));

  Serial.println("Tensões (V):");
  Serial.println("Motor: " + String(tensao_motor, 2));
  Serial.println("Auxiliar: " + String(tensao_auxiliar, 2));
  Serial.println("Placas: " + String(tensao_placas, 2));

  Serial.println("Correntes (A):");
  Serial.println("Motor: " + String(corrente_motor, 2));
  Serial.println("Entrada MPPT: " + String(corrente_entrada_mppt, 2));
  Serial.println("Saída MPPT: " + String(corrente_saida_mppt, 2));

  // Construção do JSON para envio
  StaticJsonDocument<500> doc;
  doc["topico"] = "deviceSender";
  doc["id"] = "deviceSender";
  doc["nivel"] = count++;

  doc["tensao_motor"] = tensao_motor;
  doc["tensao_auxiliar"] = tensao_auxiliar;
  doc["tensao_placas"] = tensao_placas;

  doc["corrente_motor"] = corrente_motor;
  doc["corrente_entrada_mppt"] = corrente_entrada_mppt;
  doc["corrente_saida_mppt"] = corrente_saida_mppt;

  serializeJson(doc, msgMQTT);
}

void loop() {
  if (transmittedFlag) {
    transmittedFlag = false;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      Serial.println(F("transmission finished!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);
    }

    radio.finishTransmit();

    delay(1000);

    Serial.print(F("[SX1262] Sending another packet ... "));

    adc_esp();

    Serial.println(msgMQTT);
    transmissionState = radio.startTransmit(msgMQTT);
  }
}
