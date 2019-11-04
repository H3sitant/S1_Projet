#include <Arduino.h>
#include <LibRobus.h>

#include <stdlib.h>
#include "Adafruit_TCS34725.h"

#define Bleu 1
#define Vert 2
#define Rouge 3
#define Jaune 4

float PID(float TargetSpeed);
void Avancer(float Distance,int TempsAttente);
void Tourner(int Direction, int Rotation,int TempsAttente);
void Rotation180(int NbRot,int TempsAttente);
void RadiusTurn(float Radius,int angle,int Direction,int TempsAttente);

//Initialisation des variables globales
float speed=0.4;
float L=18.350;



void setup() {
  // put your setup code here, to run once:
  	Serial.begin(9600); 
    Serial.println("test2");
    delay(5000);
    BoardInit();
   
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{
    if(ROBUS_IsBumper(3)==true)
    {
        int couleur = detectionCouleur();
    }
}
/*
==========================
Avancer
==========================
*/

 void Avancer(float Distance,int TempsAttente)
 {
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1); 
    unsigned int NbPulse=(3200*Distance)/(23.9389);
    float ActualSpeed = 0.15;
    //Serial.println(NbPulse);
    MOTOR_SetSpeed(0, ActualSpeed);
    MOTOR_SetSpeed(1, ActualSpeed);
    //MOTOR_SetSpeed(0, speed);
    //MOTOR_SetSpeed(1, speed);
    unsigned int PulseCount=ENCODER_Read(0);
    long Previous_time = millis();
    long CurrentTime = millis();
    unsigned int TimeSample = 0;
  
    do
    { 
        //Serial.println(ActualSpeed);
        TimeSample = CurrentTime-Previous_time;
        //Serial.print(TimeSample);
        //Serial.print("\t");
        //Serial.println(ActualSpeed);
        if (TimeSample>=150)
        {
          ActualSpeed=ActualSpeed*1.15;
          MOTOR_SetSpeed(1,ActualSpeed);
          MOTOR_SetSpeed(0, ActualSpeed);
          Previous_time=CurrentTime;
        }
        CurrentTime=millis();
        PulseCount=ENCODER_Read(0);
    } while (ActualSpeed<speed && ENCODER_Read(0)<3000 && PulseCount<=NbPulse);
    
      ActualSpeed = speed;
      
      //Serial.print(NbPulse);
    while(PulseCount<=NbPulse)
    {
      Serial.println(PulseCount);   
      TimeSample = CurrentTime-Previous_time;
      PulseCount=ENCODER_Read(0);
      if (PulseCount>=(NbPulse-2500) && (TimeSample)>=100)// 10cm = 1330 pulse
      {
        ActualSpeed=ActualSpeed*0.90;
      }
        if (TimeSample>=100)
        {
          MOTOR_SetSpeed(1,PID(ActualSpeed));
          MOTOR_SetSpeed(0, ActualSpeed);
          Previous_time=CurrentTime;
        }
        CurrentTime=millis();
    }
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
  
    delay(TempsAttente);
  
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1);
  }
/*
==========================
Tourner
==========================
*/
 void Tourner(int Direction, int Rotation,int TempsAttente)
 {
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  unsigned int NbPulse=((2*3200*3.14*L*Rotation*0.965)/(23.9389*360));//(2*PI*18.350(Distance entre roue)*3200*Rotation)/(360*23.9389(Nb cm par tour de roue))
  float ActualSpeed = 0.1;
  long Previous_time = millis();
  long CurrentTime = millis();
  unsigned int TimeSample = 0;
 if (Direction==0)
  {
    MOTOR_SetSpeed(1,0);
  }
  else
  {
    MOTOR_SetSpeed(0,0);
  }
  MOTOR_SetSpeed(Direction, ActualSpeed);
  unsigned int PulseCount = ENCODER_Read(Direction);
  while(PulseCount<=NbPulse)
  {    
        
        TimeSample = CurrentTime-Previous_time;
        if (TimeSample>=300 && ActualSpeed<speed)
        {
          ActualSpeed=ActualSpeed*1.15;
          MOTOR_SetSpeed(Direction,ActualSpeed);
          Previous_time=CurrentTime;
          TimeSample = 0;
        }
        CurrentTime=millis();

      PulseCount=ENCODER_Read(Direction);
  }
  MOTOR_SetSpeed(Direction, 0);
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  delay(TempsAttente);
 }
 /*
==========================
Rotation 180
==========================
*/
void Rotation180 (int NbRot, int TempsAttente)
{
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  unsigned int NbPulse=((3200*3.14*L*NbRot * 0.972)/(2*23.9389));//.972=correction robot jean-claude
  float ActualSpeed = 0.1;
  long Previous_time = millis();
  long CurrentTime = millis();
  unsigned int TimeSample = 0;
  MOTOR_SetSpeed(0, ActualSpeed);
  MOTOR_SetSpeed(1,-ActualSpeed);
  unsigned int PulseCount=ENCODER_Read(0);
  while(PulseCount<=NbPulse)
  {
       
        TimeSample = CurrentTime-Previous_time;
        if (TimeSample>=200 && ActualSpeed<speed && PulseCount<=2500)
        {
          ActualSpeed=ActualSpeed*1.2;
          MOTOR_SetSpeed(0,ActualSpeed);
          MOTOR_SetSpeed(1,-ActualSpeed);
          Previous_time=CurrentTime;
          TimeSample = 0;
        }
        if (PulseCount>=(NbPulse-2500) && TimeSample>=200)// 10cm = 1330 pulse
      {
        ActualSpeed=ActualSpeed*0.95;
        MOTOR_SetSpeed(0,ActualSpeed);
        MOTOR_SetSpeed(1,-ActualSpeed);
        Previous_time=CurrentTime;
        TimeSample = 0;

      }
        CurrentTime=millis();

      PulseCount=ENCODER_Read(0);

  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  delay(TempsAttente);

}

 /*
==========================
Boucle de controle PID
==========================
*/

 float PID(float TargetSpeed)
 {
   
  float pid = 0;
  float Kp = 0.001;
  float Ki = 1;
  float Erreur = 0;
  float sec = 7000;


    int encodeur_0 = ENCODER_Read(0);
    int encodeur_1 = ENCODER_Read(1);
    Erreur = encodeur_0 - encodeur_1;
    //Serial.println(Erreur);
    pid = (Erreur * Kp) + ((Erreur*Ki)/sec) + TargetSpeed;
    //Serial.println(pid); 
  return pid;
 }
/*
    Détection couleur
*/

int detectionCouleur ()
{
    int couleur=0;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;

    getRawData (&r,  &g,  &b,  &c);

    Serial.print("%d, %d, %d, %d",r,g,b,c);
/* 
    if(r< && r> && g< && g> && b< && b> && c< && c> ) //trouvé valeur RGBC pour bleu
    {

    }else if(r< && r> && g< && g> && b< && b> && c< && c> ) //trouvé valeur RGBC pour vert
    {

    }else if(r< && r> && g< && g> && b< && b> && c< && c> ) //trouvé valeur RGBC pour rouge
    {

    }else if(r< && r> && g< && g> && b< && b> && c< && c> ) //trouvé valeur RGBC pour jaune
    {

    }else 
    {
        couleur=0;
    }
*/
    return couleur;
}

/*
    Servomoteurs
 */

int servomoteur ()
{

    return 0;
}