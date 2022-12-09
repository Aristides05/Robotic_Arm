/*===============================================================================================*/
/*BIBLIOTECAS*/
#include <ESP32Servo.h>
#include "EEPROM.h"
#include <esp_now.h>
#include <WiFi.h>

/*===============================================================================================*/
/*MAPEAMENTO DE HARDWARE*/
                                                 /*Definições de entrada (Sensores(potenciometros))*/
#define pot_base      36
#define pot_braco     39
#define pot_cotovelo  34
#define pot_punho     35 
#define pot_punho_rot 32
#define pot_garra     33
#define btn_save       5

/*===============================================================================================*/
/*PROTIPAGEM*/
void choose_Mode();
void repetition_Mode();
void learn_Mode(); 
void repeatLearn_Mode(); 

/*===============================================================================================*/
/*OBJETOS*/
Servo servo_base;
Servo servo_braco;
Servo servo_cotovelo;
Servo servo_punho;
Servo servo_punho_rot;
Servo servo_garra;

/*===============================================================================================*/
/*DEFINIÇÕES*/
#define LIMIT_MOVS 20
#define TIME_BET_MOVS 500
#define CHANNEL 1 
#define LED 2

/*===============================================================================================*/
/*VARIAVEIS*/
uint8_t posBase[LIMIT_MOVS] = {0};
uint8_t posBraco[LIMIT_MOVS] = {0};
uint8_t posCotovelo[LIMIT_MOVS] = {0};
uint8_t posPunho[LIMIT_MOVS] = {0};
uint8_t posPunhoRot[LIMIT_MOVS] = {0};
uint8_t posGarra[LIMIT_MOVS] = {0};

static char flag_save = false;

uint8_t macAddr[6] = {0x34,0x86,0x5D,0xFD,0x39,0x04};

uint32_t previous_time = 0;
      
char flag_mode,
     flag_mode_aux,
     flag_learn,
     count_learns = 0,
     count_positions = 0;

struct positions_read
{
 uint16_t   base,
            braco,
            cotovelo,
            punho,
            punho_rot,
            garra;

 bool state_button = false;
};

struct positions_read positions;

esp_now_peer_info_t peerInfo; 
        
/*===============================================================================================*/
/*FUNÇÃO SETUP*/
void setup() 
{
  pinMode(pot_base, INPUT);
  pinMode(pot_braco, INPUT);
  pinMode(pot_cotovelo, INPUT);
  pinMode(pot_punho, INPUT);
  pinMode(pot_punho_rot, INPUT);
  pinMode(pot_garra, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(btn_save, INPUT_PULLUP);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  InitESPNow();
  
  if(!EEPROM.begin(128))
  {
    Serial.println("Falha ao iniciar EEPROM");
    Serial.println("Reiniciando em 1s");
    delay(1000);
    ESP.restart();
  }else
    Serial.println("EEPROM iniciado com sucesso");
}

/*===============================================================================================*/
/*FUNÇÃO LOOP*/
void loop() 
{
  choose_Mode();
}  // end loop 

/*===============================================================================================*/
/*FUNÇÃO CHOOSE MODE*/
void choose_Mode()
{
  if(flag_mode == 'a') flag_mode_aux = 'd';
  if(flag_mode == 'b') flag_mode_aux = 'e';
  if(flag_mode == 'c') flag_mode_aux = 'f';

  switch(flag_mode_aux)
  {
    case 'd':
      digitalWrite(LED, 1);
      repetition_Mode(); 
    break;

    case 'e': 
      digitalWrite(LED, 0);
      learn_Mode();
      Serial.println("Modo de leitura");                       //DEBUG
    break; 

    case 'f': 
      repeatLearn_Mode();
      Serial.println("Modo de repetição do que foi gravado");  //DEBUG
    break; 
  }  // end switch 
}  // end choose mode

/*===============================================================================================*/
/*FUNÇÃO REPETITION MODE*/
void repetition_Mode()
{
  positions.base = map(analogRead(pot_base), 0, 4095, 0, 180);
  positions.braco = map(analogRead(pot_braco), 0, 4095, 0, 180);
  positions.cotovelo = map(analogRead(pot_cotovelo), 0, 4095, 0, 180);
  positions.punho = map(analogRead(pot_punho), 0, 4095, 0, 180);
  positions.punho_rot = map(analogRead(pot_punho_rot), 0, 4095, 0, 180);
  positions.garra = map(analogRead(pot_garra), 0, 4095, 0, 180);

  esp_now_send(macAddr, (uint8_t *) &positions, sizeof(positions));
}  // end repetition mode

/*===============================================================================================*/
/*FUNÇÃO LEARN MODE*/
void learn_Mode()
{
  positions.base = map(analogRead(pot_base), 0, 4095, 0, 180);
  positions.braco = map(analogRead(pot_braco), 0, 4095, 0, 180);
  positions.cotovelo = map(analogRead(pot_cotovelo), 0, 4095, 0, 180);
  positions.punho = map(analogRead(pot_punho), 0, 4095, 0, 180);
  positions.punho_rot = map(analogRead(pot_punho_rot), 0, 4095, 0, 180);
  positions.garra = map(analogRead(pot_garra), 0, 4095, 0, 180);

  if(!digitalRead(btn_save)) flag_save = true;
  if(digitalRead(btn_save) && flag_save)
  {
    positions.state_button = true; 
    flag_save = false;
  }
  
  esp_now_send(macAddr, (uint8_t *) &positions, sizeof(positions));
  positions.state_button = false;
  
} // end learn mode
/*===============================================================================================*/
/*FUNÇÃO REPEAT LEARN MODE*/
void repeatLearn_Mode()
{
}

/*===============================================================================================*/
/*FUNÇÃO INICIALIZA / CONFIGURA ESP NOW*/
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
    esp_now_register_recv_cb(OnDataRecv);
}

/*===============================================================================================*/
/*FUNÇÃO DATA SEND*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("Status: "); 
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

/*===============================================================================================*/
/*FUNÇÃO DATA RECV*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) 
{
    memcpy(&flag_mode, incomingData, sizeof(incomingData));
}
