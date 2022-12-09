/*===============================================================================================*/
/*BIBLIOTECAS*/
#include <ESP32Servo.h>
#include "EEPROM.h"
#include <esp_now.h>
#include <WiFi.h>

/*===============================================================================================*/
/*MAPEAMENTO DE HARDWARE*/
                                                                /*Definições de saida (atuadores)*/ 
#define base      13
#define braco     12
#define cotovelo  14
#define punho     27 
#define punho_rot 26
#define garra     25

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
#define TIME_BET_MOVS 100
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

uint8_t macAddr[6] = {0xC8,0xF0,0x9E,0x9F,0x8D,0x70};

uint32_t previous_time = 0;
      
char flag_mode,
     flag_mode_aux,
     flag_learn,
     count_learns = 0,
     count_positions = 0;

bool machine_state = 0,
     btn_save;

struct positions_received
{
  uint16_t base_pos,
           braco_pos,
           cotovelo_pos,
           punho_pos,
           punho_rot_pos,
           garra_pos;

  bool state_button = false;
};

struct positions_received positions;

esp_now_peer_info_t peerInfo;

/*===============================================================================================*/
/*FUNÇÃO SETUP*/
void setup() 
{
  pinMode(base, OUTPUT);
  pinMode(braco, OUTPUT);
  pinMode(cotovelo, OUTPUT);
  pinMode(punho, OUTPUT);
  pinMode(punho_rot, OUTPUT);
  pinMode(garra, OUTPUT);
  pinMode(LED, OUTPUT);

  servo_base.attach(base);
  servo_braco.attach(braco);
  servo_cotovelo.attach(cotovelo);
  servo_punho.attach(punho);
  servo_punho_rot.attach(punho_rot);
  servo_garra.attach(garra);

  Serial.begin(115200);
  Serial2.begin(9600);
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
  if(machine_state)
  {
    choose_Mode();
  }else if(!machine_state)
  {
    if(Serial2.available())
    {
      if(Serial2.read() == 'l')
        machine_state = true;
    }
  }   
}  // end loop 

/*===============================================================================================*/
/*FUNÇÃO CHOOSE MODE*/
void choose_Mode()
{
  if(Serial2.available())
  {
    flag_mode = Serial2.read();

    if(flag_mode != 'd')
    {
      esp_now_send(macAddr, (uint8_t *) &flag_mode, sizeof(flag_mode));

      if(flag_mode == 'a') flag_mode_aux = 'd';
      if(flag_mode == 'b') flag_mode_aux = 'e';
      if(flag_mode == 'c') flag_mode_aux = 'f';

    }else if(flag_mode == 'd')
      machine_state = false;
  }

  if(machine_state)
    switch(flag_mode_aux)
    {
      case 'd':
        repetition_Mode(); 
        Serial.println("Modo de repetição");  //DEBUG
      break;

      case 'e': 
        learn_Mode();
        Serial.println("Modo de gravação");  //DEBUG
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
  servo_base.write(positions.base_pos);
  servo_braco.write(positions.braco_pos);
  servo_cotovelo.write(positions.cotovelo_pos);
  servo_punho.write(positions.punho_pos);
  servo_punho_rot.write(positions.punho_rot_pos);
  servo_garra.write(positions.garra_pos);
}  // end repetition mode

/*===============================================================================================*/
/*FUNÇÃO LEARN MODE*/
void learn_Mode()
{
  servo_base.write(positions.base_pos);
  servo_braco.write(positions.braco_pos);
  servo_cotovelo.write(positions.cotovelo_pos);
  servo_punho.write(positions.punho_pos);
  servo_punho_rot.write(positions.punho_rot_pos);
  servo_garra.write(positions.garra_pos);
  Serial.println(positions.base_pos);

  if(Serial2.available())
    flag_learn = Serial2.read();

  if(flag_learn == 'g')
  {
    Serial.println("botao pressionado, posições salvas");
    delay(4000);
    flag_learn = 'n';
    

    EEPROM.writeByte(count_learns,   positions.base_pos);
    EEPROM.writeByte(count_learns+1, positions.braco_pos);
    EEPROM.writeByte(count_learns+2, positions.cotovelo_pos);
    EEPROM.writeByte(count_learns+3, positions.punho_pos);
    EEPROM.writeByte(count_learns+4, positions.punho_rot_pos);
    EEPROM.writeByte(count_learns+5, positions.garra_pos);

    count_learns+=6;
    count_positions++;
  } 
} 

/*===============================================================================================*/
/*FUNÇÃO REPEAT LEARN MODE*/
void repeatLearn_Mode()
{ 
  for(byte flag = 0; flag <= count_positions; flag++)
  {
    servo_base.write(EEPROM.readByte(flag));
    delay(TIME_BET_MOVS);
    servo_braco.write(EEPROM.readByte(flag+1));
    delay(TIME_BET_MOVS);
    servo_cotovelo.write(EEPROM.readByte(flag+2));
    delay(TIME_BET_MOVS);
    servo_punho.write(EEPROM.readByte(flag+3));
    delay(TIME_BET_MOVS);
    servo_punho_rot.write(EEPROM.readByte(flag+4));
    delay(TIME_BET_MOVS);
    servo_garra.write(EEPROM.readByte(flag+5));
    delay(TIME_BET_MOVS);
    Serial.println(EEPROM.readByte(flag));
    Serial.println(EEPROM.readByte(flag+1));
    Serial.println(EEPROM.readByte(flag+2));
    Serial.println("=====================");
  }
}

/*===============================================================================================*/
/*FUNÇÃO INIT ESP NOW*/
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
/*FUNÇÃO SEND CALLBACK*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("Status: "); 
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

/*===============================================================================================*/
/*FUNÇÃO RECV CALLBACK*/
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) 
{
  memcpy(&positions, incomingData, sizeof(incomingData));
}
