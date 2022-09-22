#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1
esp_now_peer_info_t peerInfo;

uint8_t var_teste = 90, 
        var_teste2;

uint8_t macAddr[6] = {/*endereço mac do outro Esp*/};

void setup()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());

    InitESPNow();

    peerInfo.channel = CHANNEL; 
    peerInfo.encrypt = 0;
    memcpy(peerInfo.peer_addr, macAddr, sizeof(macAddr));  // Destino  Origem  Tamanho em bytes
    
    if(esp_now_add_peer(&peerInfo) == ESP_OK)
        Serial.println("Emparelhamento realizado com sucesso");
    else
    {
        Serial.println("Falha ao emparelhar");
        Serial.println("Reiniciando...");
        delay(1000);
        ESP.restart();
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
    esp_err_t result = esp_now_send(macAddr, (uint8_t *) &var_teste, sizeof(var_teste));  // endereço  /  buffer de dados  /  tamanho em bytes 
   
  if (result == ESP_OK)
    Serial.println("Enviado com sucesso");
  else
    Serial.println("Erro ao enviar dados");
}

void InitESPNow()
{
    if(esp_now_init() == ESP_OK)
        Serial.println("ESP Now Iniciado com sucesso");
    else
    {
        Serial.println("Falha ao iniciar ESP Now");
        Serial.println("Reiniciando...");
        delay(1000);
        ESP.restart();
    }
}

// Callback quando os dados são enviados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nStatus dos ultimos dados enviados: \t");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Successo" : "Erro");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback quando os dados são recebidos 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&var_teste2, incomingData, sizeof(var_teste2));
  Serial.print("Bytes recebido: ");
  Serial.println(len);

  /*
  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.hum;
  incomingPres = incomingReadings.pres;
  */
}