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
int detectionCouleur();
void servomoteurPrendre ();
void servomoteurLacher ();
float detecteurLigne(float TargetSpeed);


//Initialisation des variables globales
float speed=0.4;
float L=18.350;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void setup() {
  // put your setup code here, to run once:
  	Serial.begin(9600); 
    Serial.println("test1");
    delay(5000);
    BoardInit();
    pinMode(41,OUTPUT);


  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1); // halt!
  }
  
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{
    if(ROBUS_IsBumper(3)==true)
    {
      Serial.print("detecteur ligne");
      float newSpeed = detecteurLigne(0.34);
    }
    if(ROBUS_IsBumper(2)==true)
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

  Serial.println(r);
  Serial.println(g);
  Serial.println(b);
  Serial.println(c);
 
    /*if(r<100 && r>20 && g<100 && g>20 && b>100 && c>200) //trouvé valeur RGBC pour bleu
    {
        couleur=Bleu;
    }*/
    /*else if(r<90 && r>10 && g<120 && g>40 && b<115 && b>35 && c<260 && c>180 ) //trouvé valeur RGBC pour vert
    {
      couleur=Vert;
    }*/
    if(r>75 && r<180 && g<70 && g>40 && b<80 && b>40 && c>180 && c<450) //trouvé valeur RGBC pour rouge
    {
      couleur=Rouge;
    }
    else if(r>140 && r<200 && g>110 && g<180 && b>60 && b<80 && c<600 && c>350) //trouvé valeur RGBC pour jaune
    {
      couleur=Jaune;
    }
    else 
    {
        couleur=0;
    }
  Serial.println(couleur);
  delay(1000);
  return couleur;
}

/*
==========================
Servomoteurs prendre
==========================
 */

void servomoteurPrendre ()
{
  SERVO_Enable(0);
  SERVO_SetAngle(0, 180);
}

/*
==========================
Servomoteurs lacher
==========================
 */

void servomoteurLacher ()
{
  SERVO_SetAngle(0, 0);
  SERVO_Disable(0);
}

/*
==========================
Dectecteur de ligne
==========================
*/
 float detecteurLigne(float TargetSpeed)
 {
  float vitesseRoue=0;
  digitalWrite(35,HIGH);
  float tension = analogRead(A8)*5.0/1023.0;
  Serial.println(tension);
  delay(500);

  Serial.println(analogRead(60));
  //En fonction de détection de Blanc 
  if (tension<0.5)//option 1(Aucun) 0.00 
  {

  }
  else if(tension< 1)//option 2(X3) 0.71
  {

  }
  else if(tension< 1.9)//option 3(X2) 1.51
  {

  }
  else if(tension< 2.40)//option 4(X2,X3) 2.14
  {

  }
  else if(tension< 3)//option 5(X1) 2.85
  {

  }
  else if(tension< 3.9)//option 6(X1,X3) 3.57
  {

  }
  else if(tension< 4.5)//option 7(X1,X2) 4.28
  {

  }
  else//option 8(X1,X2,X3) 5
  {

  }


  /*int encodeur_0 = ENCODER_Read(0);
  int encodeur_1 = ENCODER_Read(1);
  Erreur = encodeur_0 - encodeur_1;
  //Serial.println(Erreur);
  pid = (Erreur * Kp) + ((Erreur*Ki)/sec) + TargetSpeed;
  //Serial.println(pid); 
  return pid;*/
  delay(1000);
  return vitesseRoue;
 }
 
 /*
 ==========================
 IR sensor
 ==========================
  */
 float IR_Sensor()
 {
	 uint16_t distance=ROBUS_ReadIR(0);    // reads the value of the sharp sensor
	 //Serial.println(val);            // prints the value of the sensor to the serial monitor
	 delay(200);
   return;
                       // wait for this much time before printing next value
 }