#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1  // Definição do canal

uint8_t macAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //Colocar MacAddress do ESP

void setup() 
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);  // ESP em modo station

  Serial.print("Mac Address in Station: "); 
  Serial.println(WiFi.macAddress());

  InitESPNow();

  esp_now_peer_info_t peer; // Variavel para add inf's sobre o peer
  peer.channel = CHANNEL;  // Canal utilizado
  peer.encrypt = 0; // Sem criptografia
  memcpy(peer.peer_addr, macAddr, sizeof(macAddr)); //Copia o endereço do array para a estrutura 
                                                               // memcpy pede 3 parametros / destino, origem e tamanho em bytes
  esp_now_add_peer(&peer); //Adiciona o peer

  esp_now_register_send_cb(OnDataSent); //Registro da função de callback, que ira ser escrita mais abaixo
  send();  //função criada para enviar dados
}

void InitESPNow() 
{
  if (esp_now_init() == ESP_OK) 
    Serial.println("ESPNow Iniciado Successo");
  else 
  {
    Serial.println("Falha ao iniciar ESPNow");
    Serial.println("Reiniciando ESP em 1s");
    delay(1000);
    ESP.restart();
  }
}

void send()
{
  uint8_t values[gpioCount];
  uint8_t broadcast[] = {0xFF, 0xFF,0xFF,0xFF,0xFF,0xFF};

  for(int i=0; i<gpioCount; i++)
  {
    values[i] = digitalRead(gpios[i]);
  }

  esp_err_t result = esp_now_send(broadcast, (uint8_t*) &values, sizeof(values)); // função send pede 3 parametros, endereço, 
                                                                                  //dados (buffer), tamanho em bytes
  Serial.print("Send Status: ");

  if (result == ESP_OK) 
    Serial.println("Success");
  else
    Serial.println("Error");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  char macStr[18];
  //Copiamos o Mac Address destino para uma string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Mostramos o Mac Address que foi destino da mensagem
  Serial.print("Sent to: "); 
  Serial.println(macStr);
  Serial.print("Status: "); 
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  send();
}

void loop() 
{
}