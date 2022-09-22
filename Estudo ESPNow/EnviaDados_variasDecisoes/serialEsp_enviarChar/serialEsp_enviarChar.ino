#include <esp_now.h>
#include <WiFi.h>

#define LED 2
#define CHANNEL 1

uint8_t flag;
uint8_t macAddr[6] = {0x34,0x86,0x5D,0xFD,0x39,0x04};

esp_now_peer_info_t peerInfo; 

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  InitESPNow();

  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = 0;
  memcpy(peerInfo.peer_addr, macAddr, sizeof(macAddr));

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
}

void loop() 
{
  if(Serial.available())
  {
    flag = Serial.read();
    esp_err_t result = esp_now_send(macAddr, (uint8_t *) &flag, sizeof(flag));
    
    if(result == ESP_OK)
      Serial.println("Enviado com sucesso");
    else
      Serial.println("Erro ao enviar dados");   
  } 
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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Status: "); 
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
