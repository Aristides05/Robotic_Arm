/*===============================================================================================*/
/*BIBLIOTECAS*/
#include <Arduino.h>
#include <ESP32Servo.h>
#include "EEPROM.h"

/*===============================================================================================*/
/*MAPEAMENTO DE HARDWARE*/
                                                                /*Definições de saida (atuadores)*/ 
#define base      36
#define braco     39
#define cotovelo  34
#define punho     35 
#define punho_rot 32
#define garra     33 

                                                /*Definições de entrada (Sensores(potenciometros))*/
#define pot_base      13
#define pot_braco     12
#define pot_cotovelo  14
#define pot_punho     27 
#define pot_punho_rot 26
#define pot_garra     25

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

/*===============================================================================================*/
/*VARIAVEIS*/
uint8_t posBase[LIMIT_MOVS] = {0};
uint8_t posBraco[LIMIT_MOVS] = {0};
uint8_t posCotovelo[LIMIT_MOVS] = {0};
uint8_t posPunho[LIMIT_MOVS] = {0};
uint8_t posPunhoRot[LIMIT_MOVS] = {0};
uint8_t posGarra[LIMIT_MOVS] = {0};

uint32_t previous_time = 0;
      
char flag_mode,
     flag_learn,
     count_learns = 0,
     count_positions = 0;
        
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

  pinMode(pot_base, INPUT);
  pinMode(pot_braco, INPUT);
  pinMode(pot_cotovelo, INPUT);
  pinMode(pot_punho, INPUT);
  pinMode(pot_punho_rot, INPUT);
  pinMode(pot_garra, INPUT);

  servo_base.attach(base);
  servo_braco.attach(braco);
  servo_cotovelo.attach(cotovelo);
  servo_punho.attach(punho);
  servo_punho_rot.attach(punho_rot);
  servo_garra.attach(garra);

  Serial.begin(115200);
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
  if(Serial.available())
    flag_mode = Serial.read();

  switch(flag_mode)
  {
    case 'a':
      repetition_Mode(); 
      Serial.println("Modo de repetição");                     //DEBUG
    break;

    case 'b': 
      learn_Mode();
      Serial.println("Modo de leitura");                       //DEBUG
    break; 

    case 'c': 
      repeatLearn_Mode();
      Serial.println("Modo de repetição do que foi gravado");  //DEBUG
    break; 
  }  // end switch 
}  // end choose mode

/*===============================================================================================*/
/*FUNÇÃO REPETITION MODE*/
void repetition_Mode()
{
  servo_base.write(map(analogRead(pot_base), 0, 4095, 0, 180));
  servo_braco.write(map(analogRead(pot_braco), 0, 4095, 0, 180));
  servo_cotovelo.write(map(analogRead(pot_cotovelo), 0, 4095, 0, 180));
  servo_punho.write(map(analogRead(pot_punho), 0, 4095, 0, 180));
  servo_punho_rot.write(map(analogRead(pot_punho_rot), 0, 4095, 0, 180));
  servo_garra.write(map(analogRead(pot_garra), 0, 4095, 0, 180));
}  // end repetition mode

/*===============================================================================================*/
/*FUNÇÃO LEARN MODE*/
void learn_Mode()
{
  servo_base.write(map(analogRead(pot_base), 0, 4095, 0, 180));
  servo_braco.write(map(analogRead(pot_braco), 0, 4095, 0, 180));
  servo_cotovelo.write(map(analogRead(pot_cotovelo), 0, 4095, 0, 180));
  servo_punho.write(map(analogRead(pot_punho), 0, 4095, 0, 180));
  servo_punho_rot.write(map(analogRead(pot_punho_rot), 0, 4095, 0, 180));
  servo_garra.write(map(analogRead(pot_garra), 0, 4095, 0, 180));

  if(Serial.available() != 0)
  {
    flag_learn = Serial.read();

    if(flag_learn == 'l' || flag_learn == 'L')
    {
      posBase[count_learns] = map(analogRead(pot_base), 0, 4095, 0, 180);
      posBraco[count_learns] = map(analogRead(pot_braco), 0, 4095, 0, 180);
      posCotovelo[count_learns] = map(analogRead(pot_cotovelo), 0, 4095, 0, 180);
      posPunho[count_learns] = map(analogRead(pot_punho), 0, 4095, 0, 180);
      posPunhoRot[count_learns] = map(analogRead(pot_punho_rot), 0, 4095, 0, 180);
      posGarra[count_learns] = map(analogRead(pot_garra), 0, 4095, 0, 180);

      EEPROM.writeByte(count_learns, posBase[count_learns]);
      EEPROM.writeByte(count_learns+1, posBraco[count_learns]);
      EEPROM.writeByte(count_learns+2, posCotovelo[count_learns]);
      EEPROM.writeByte(count_learns+3, posPunho[count_learns]);
      EEPROM.writeByte(count_learns+4, posPunhoRot[count_learns]);
      EEPROM.writeByte(count_learns+5, posGarra[count_learns]);

      count_learns++;
    }
  }
}  

/*===============================================================================================*/
/*FUNÇÃO REPEAT LEARN MODE*/
void repeatLearn_Mode()
{
  delay(1000);
  count_positions = count_learns;
  count_learns = 0;
  
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
  }
}