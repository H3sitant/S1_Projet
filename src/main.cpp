#include <Arduino.h>
#include <LibRobus.h>
#include<math.h>

#include <stdlib.h>
#include "Adafruit_TCS34725.h"

#define Vert 1
#define Jaune 2
#define Rouge 3

#define ligne 1
#define Millieux 2
#define Vide 0

//declaration fonction
float PID(float TargetSpeed);
void Avancer(float Distance,int TempsAttente,float TargetSpeed,int arret);
void Tourner(int Direction, int Rotation,int TempsAttente,int arret,float TargetSpeed);
void Rotation(int angle,int TempsAttente);
void RadiusTurn(float Radius,int angle,int Direction,int TempsAttente);
int detectionCouleur();
void servomoteurPrendre ();
void servomoteurLacher ();
int detecteurLigne(float TargetSpeed,int type);
void TrouverBallon();
uint16_t Distance (uint8_t capteurId);
void Rotation1 (int Angle, int TempsAttente,int cote);


//Initialisation des variables globales
float speed=0.12;
float L=18.350;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//midi
void partition(int couleur);
void trouver_speed();

void setup() 
{
  // put your setup code here, to run once:
  	Serial.begin(9600); 
    Serial.println("test2");
    delay(1000);
    BoardInit();
    pinMode(41,OUTPUT);
    pinMode(39,OUTPUT);


  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    
  }
  SERVO_SetAngle(1, 150);
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{

/*
======================
Code Robot esclave
======================
 */
  
  if(ROBUS_IsBumper(1)==true)
  {
    
    Serial.println(1);
   partition(Vert);
  }
  else if(ROBUS_IsBumper(2)==true)
  {
    
    Serial.println(2);
    partition(Jaune);
  }
  else if(ROBUS_IsBumper(3)==true)
  {
    
    Serial.println(3);
    partition(Rouge);
  }
  else if(ROBUS_IsBumper(0)==true)
  {
    long temps=millis();
    long tempsm=micros();
    while(millis()-temps<16675)
    {
      if((millis()-temps)%1000>480 && (millis()-temps)%1000<520)
      {
        tempsm=micros();
        while (micros()-tempsm<60000)
        {
          digitalWrite(39,HIGH);
          delayMicroseconds(1200);
          digitalWrite(39,LOW);
          delayMicroseconds(1200);
        } 
      }
      temps=millis();
      detecteurLigne(speed,1);
      Serial.println(millis()-temps);
    }
  }
}
/*
=========================
Traitement déplacement en fonction de la partition
========================
*/
void partition (int couleur)
{
  while( detectionCouleur()!=couleur)
  {
    detecteurLigne(speed,1);
  }
  Avancer(5.7*2.54/2+9.53,0,-speed,1);
  delay(1000);
  Rotation1(85, 1000,0);
  Avancer(7,0,speed,1);
  //Envoyer pres
  delay(8000);
  long temps=millis();
  while(millis()-temps<16675)
  {
    if((millis()-temps)%1000==675)AX_BuzzerON(500,50);
    detecteurLigne(speed,1);
  }
   Rotation1(180,20,0);
  while( detectionCouleur()!=couleur)
  {
    detecteurLigne(speed,1);
  }
  Avancer(5.7*2.54/2+9.53,0,-speed,1);
  Rotation1(90, 1000, 1);
  if(couleur!=Vert)
  while( detectionCouleur()!=Vert)
  {
    detecteurLigne(speed,1);
  }
  Avancer(5.7*2.54*2+9.53,0,-speed,1);
  Rotation1(198,20,0);
}

/*
==========================
Avancer
==========================
*/

 void Avancer(float Distance,int TempsAttente,float TargetSpeed,int arret)
 {
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1); 
    unsigned int NbPulse=(3200*Distance)/(23.9389);
    MOTOR_SetSpeed(0, TargetSpeed);
    MOTOR_SetSpeed(1, TargetSpeed);
    unsigned int PulseCount=abs(ENCODER_Read(0));
    long Previous_time = millis();
    long CurrentTime = millis();
    unsigned int TimeSample = 0;

   while(PulseCount<=NbPulse)
    {
      TimeSample = CurrentTime-Previous_time;
      PulseCount = abs(ENCODER_Read(0));
      //Serial.println(PulseCount); 

         if (TimeSample>=100)
        {
          MOTOR_SetSpeed(1,PID(TargetSpeed));
          MOTOR_SetSpeed(0, TargetSpeed);
          Previous_time=CurrentTime;
        }
        CurrentTime=millis();
    }
    if (arret==1){
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
    }
  
    delay(TempsAttente);
  
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1);
    //Serial.println("Fin avancer");
  }
/*
==========================
Tourner
==========================
*/
 void Tourner(int Direction, int Rotation,int TempsAttente,int arret,float TargetSpeed)
 {
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  unsigned int NbPulse=((2*3200*3.14*L*Rotation*0.965)/(23.9389*360));//(2*PI*18.350(Distance entre roue)*3200*Rotation)/(360*23.9389(Nb cm par tour de roue))
  float ActualSpeed = 0.2;
  if (TargetSpeed != 0.0)
  {
    ActualSpeed=TargetSpeed;
  }
 if (Direction==0)
  {
    MOTOR_SetSpeed(1,0);
  }
  else
  {
    MOTOR_SetSpeed(0,0);
  }
  MOTOR_SetSpeed(Direction, ActualSpeed);
  unsigned int PulseCount = abs(ENCODER_Read(Direction));
  //Serial.println(NbPulse);
  while(PulseCount<=NbPulse)
  {    
    MOTOR_SetSpeed(Direction,ActualSpeed);
    PulseCount=abs(ENCODER_Read(Direction));
    //Serial.println(PulseCount);
  }
  if (arret==1){
  MOTOR_SetSpeed(Direction, 0);}

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  delay(TempsAttente);
  //Serial.println("Fin de Tourner");
 }
 /*
==========================
Rotation 180
==========================
*/
void Rotation (int Angle, int TempsAttente)
{
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  unsigned int NbPulse=((3200*3.14*L*Angle* 0.972)/(360*23.9389));//.972=correction robot jean-claude
  float ActualSpeed = 0.1;
  long Previous_time = millis();
  long CurrentTime = millis();
  unsigned int TimeSample = 0;
  MOTOR_SetSpeed(0, ActualSpeed);
  MOTOR_SetSpeed(1,-ActualSpeed);
  unsigned int PulseCount=ENCODER_Read(0);
  //Serial.println(NbPulse);
  while(PulseCount<=NbPulse)
  {
       
        TimeSample = CurrentTime-Previous_time;
        if (TimeSample>=200 && ActualSpeed<speed && PulseCount<NbPulse/2)
        {
          ActualSpeed=ActualSpeed*1.2;
          MOTOR_SetSpeed(0,ActualSpeed);
          MOTOR_SetSpeed(1,-ActualSpeed);
          Previous_time=CurrentTime;
          TimeSample = 0;
        }
        if (PulseCount>=(NbPulse/2) && TimeSample>=200 && ActualSpeed>0.15)// 10cm = 1330 pulse
      {
        ActualSpeed=ActualSpeed*0.75;
        MOTOR_SetSpeed(0,ActualSpeed);
        MOTOR_SetSpeed(1,-ActualSpeed);
        Previous_time=CurrentTime;
        TimeSample = 0;
        Serial.println(ActualSpeed);

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
===============
Rotation avec coté
=================
 */
void Rotation1 (int Angle, int TempsAttente,int cote)
{
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  unsigned int NbPulse=((3200*3.14*L*Angle* 0.972)/(360*23.9389));//.972=correction robot jean-claude
  float ActualSpeed = 0.1;
  long Previous_time = millis();
  long CurrentTime = millis();
  unsigned int TimeSample = 0;
  if(cote==1)
  {
    ActualSpeed=-ActualSpeed;
  }
  MOTOR_SetSpeed(0, ActualSpeed);
  MOTOR_SetSpeed(1,-ActualSpeed);
  unsigned int PulseCount=abs(ENCODER_Read(0));
  //Serial.println(NbPulse);
  while(PulseCount<=NbPulse)
  {
       
        TimeSample = CurrentTime-Previous_time;
        if (TimeSample>=200 && ActualSpeed<speed && PulseCount<NbPulse/2)
        {
          ActualSpeed=ActualSpeed*1.2;
          MOTOR_SetSpeed(0,ActualSpeed);
          MOTOR_SetSpeed(1,-ActualSpeed);
          Previous_time=CurrentTime;
          TimeSample = 0;
        }
        if (PulseCount>=(NbPulse/2) && TimeSample>=200 && ActualSpeed>0.15)// 10cm = 1330 pulse
      {
        ActualSpeed=ActualSpeed*0.75;
        MOTOR_SetSpeed(0,ActualSpeed);
        MOTOR_SetSpeed(1,-ActualSpeed);
        Previous_time=CurrentTime;
        TimeSample = 0;
        Serial.println(ActualSpeed);

      }
        CurrentTime=millis();

      PulseCount=abs(ENCODER_Read(0));

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
==========================
Détection couleur
==========================
*/

int detectionCouleur ()
{
  int couleur=0;
  uint16_t r=0;
  uint16_t g=0;
  uint16_t b=0;
  uint16_t c=0;

  tcs.getRawData(&r,  &g,  &b,  &c);

  /*Serial.println(r);
  Serial.println(g);
  Serial.println(b);
  Serial.println(c);
  delay(1000);*/
 
    if(r>30 && r<45 && g>50 && g<70 && b>55 && b<75 && c>165 && c<250) //trouvé valeur RGBC pour bleu
    {
        couleur=Vert;
        
    }
    else if(r>100 && r<150 && g>90 && g<140 && b>70 && b<=100 && c>250) //trouvé valeur RGBC pour vert
    {
      couleur=Jaune;
    }
    else if(r>70 && r<100 && g>30 && g<80 && b>45 && b<70 && c>180 && c<240) //trouvé valeur RGBC pour rouge
    {
      couleur=Rouge;
    }
    else 
    {
        couleur=0;
    }
  //Serial.println(couleur);
  return couleur;
}

/*
==========================
Servomoteurs prendre
==========================
 */

void servomoteurPrendre ()
{
  SERVO_Enable(1);
  SERVO_SetAngle(1,30);
  delay(2000);
  SERVO_Disable(1);
}

/*
==========================
Servomoteurs lacher
==========================
 */

void servomoteurLacher ()
{
  SERVO_Enable(1);
  SERVO_SetAngle(1, 150);
  delay(2000);
  SERVO_Disable(1);
}

/*
==========================
Dectecteur de ligne
==========================
*/
 int detecteurLigne(float TargetSpeed,int type)
 {
  int Detect=0;
  float tension = analogRead(A8)*5.0/1023.0;
  Serial.println(tension);

 

  //Serial.println(analogRead(60));
  //En fonction de détection de Blanc 
  if (tension<0.5) //option 1(Aucun) 0.00 
  {
    Detect=Millieux;
  }
  else if(tension < 1 && tension >0.5) //option 2(X3) 0.71
  {
    Detect=ligne;
  }
  else if(tension< 1.9 && tension > 1) //option 3(X2) 1.51
  {
    Detect=ligne;
  }
  else if(tension< 2.40 && tension > 1.9)  //option 4(X2,X3) 2.14
  {
    if (type==1)
    {
      MOTOR_SetSpeed(1,0);
      MOTOR_SetSpeed(0,0);
      Tourner(1,5,0,0,-TargetSpeed*1.5);
    }
    Detect=ligne;
  }
  else if(tension< 3 && tension > 2.40) //option 5(X1) 2.85
  {
    Detect=ligne;
  }
  else if(tension< 3.9 && tension > 3) //option 6(X1,X3) 3.57
  {
   
   Detect=ligne;
  }
  else if(tension< 4.5 && tension >3.9) //option 7(X1,X2) 4.28
  {
    if (type==1)
    {
      MOTOR_SetSpeed(1,0);
      MOTOR_SetSpeed(0,0);
      Tourner(0,5,0,0,-TargetSpeed*1.5);
    }
    Detect=ligne;
  }
  else //option 8(X1,X2,X3) 5
  {
    Detect=Vide;
  }
  Avancer(0.25,0,-TargetSpeed,0);

  
  return Detect;
 }


 
 /*
 ==========================
 Trouver ballon avec IR sensor
 ==========================
*/

void TrouverBallon()
{
  uint16_t distance = 3000;
  int angle = 25;

  for(int i=0;i<=5;i++)
    {
      Rotation1(5,0,0);
        if (distance>=Distance(1))
        {
          distance=Distance(1);
          angle=25+5*(i+1);
        }
      Serial.println(distance);
      Serial.println(angle);
    }
  for(int i=0;i<=10;i++)
  {
    Rotation1(5,0,1);
      if (distance>=Distance(1))
      {
        distance=Distance(1);
        angle=55-(i+1)*5;
      }
    Serial.println(distance);
    Serial.println(angle);
  }

  Rotation1(angle,0,0);
  Avancer(distance,300,speed,1);
  servomoteurPrendre ();
  //Rotation(180,0);
  //Avancer(35,500,0.3,1);
}

uint16_t Distance (uint8_t capteurId)
{
    int16_t TensionIR = ROBUS_ReadIR(capteurId);
    if (TensionIR != 0)
    {
      int distance = (6787 / (TensionIR - 3)) -4;
      return distance;
    }
    return 0;
}