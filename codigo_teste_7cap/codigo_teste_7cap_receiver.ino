#include <RadioLib.h>
#include <ArduinoJson.h>

SX1262 radio = new Module(8, 14, 12, 13);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet is received by the module
#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {
  receivedFlag = true;
}

void setup() {
  Serial.begin(9600);

  // initialize SX1262 with default settings
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

  // set the function that will be called when new packet is received
  radio.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1262] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
}

void loop() {
  if (receivedFlag) {
    receivedFlag = false;

    String str;
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("\n==================== NOVO PACOTE ===================="));
      Serial.println(F("[SX1262] Pacote recebido com sucesso!"));

      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, str);

      if (!error) {
        Serial.println(F("[SX1262] Conteúdo JSON formatado:"));
        serializeJsonPretty(doc, Serial);
        Serial.println();

        Serial.println(F("\n--- Visualização Detalhada ---"));
        Serial.print(F("Tensão Placas: ")); Serial.println(doc["tensao_placas"].as<float>());
        Serial.print(F("Tensão Auxiliar: ")); Serial.println(doc["tensao_circuito_aux"].as<float>());
        Serial.print(F("Tensão Motor: ")); Serial.println(doc["tensao_motor"].as<float>());
        Serial.print(F("Corrente Entrada MPPT: ")); Serial.println(doc["corrente_entrada_mppt"].as<float>());
        Serial.print(F("Corrente Saída MPPT: ")); Serial.println(doc["corrente_saida_mppt"].as<float>());
      } else {
        Serial.print(F("[Erro] Falha ao interpretar JSON: "));
        Serial.println(error.c_str());
        Serial.print(F("Dados brutos recebidos: "));
        Serial.println(str);
      }

      Serial.print(F("\n[SX1262] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));

      Serial.print(F("[SX1262] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));

      Serial.print(F("[SX1262] Erro de frequência:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));

      Serial.println(F("=====================================================\n"));

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println(F("[SX1262] Erro de CRC - pacote corrompido!"));

    } else {
      Serial.print(F("[SX1262] Erro desconhecido, código "));
      Serial.println(state);
    }
  }
}
