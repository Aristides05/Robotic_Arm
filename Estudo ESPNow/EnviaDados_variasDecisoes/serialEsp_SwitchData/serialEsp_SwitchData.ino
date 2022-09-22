#include <esp_now.h>
#include <WiFi.h>

#define LED 2
uint8_t flag; 

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
  Serial.println(flag);
  switch(flag)
  {
    case 'a':
      digitalWrite(LED, 1);
      break;
    case 'b':
      digitalWrite(LED, 0);
      break;
    case 'c':  
     flag = c
      digitalWrite(LED, 1);
      delay(500);
      digitalWrite(LED, 0);
      delay(500);
      break;
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

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) 
{
  memcpy(&flag, incomingData, sizeof(incomingData));
  //teste = incomingData; deu errado
}
