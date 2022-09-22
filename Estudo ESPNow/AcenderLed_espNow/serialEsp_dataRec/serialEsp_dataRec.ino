#include <esp_now.h>
#include <WiFi.h>

#define LED 2
bool teste; 

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(LED, OUTPUT);
}

void loop()
{
  
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

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) 
{
  memcpy(&teste, incomingData, sizeof(teste));
  //teste = incomingData; deu errado
  digitalWrite(LED, teste);
}
