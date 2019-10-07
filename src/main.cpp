#include <Arduino.h>
#include <LibRobus.h>

long PID(long PreviousTime,float TargetSpeed);
void Avancer(float Distance,int TempsAttente);
void Tourner(int Direction, int Rotation,int TempsAttente);
void Rotation180(int NbRot,int TempsAttente);
void RadiusTurn(float Radius,int angle,int Direction,int TempsAttente);

//Initialisation des variables globales
float speed=0.4;
float L=18.350;

float pid = 0;
float Kp = 0.001;
float Ki = 1;
float Erreur = 0;
float sec = 7000;
unsigned int dT = 100;

void setup() {
  // put your setup code here, to run once:
  	Serial.begin(9600);
    delay(5000);
    BoardInit();
}

void loop() {
/*
Direction:  0 = Rotation Horraire     1 = Rotation AntiHorraire
*/

  /*Avancer(40,2000); //Appel de la fonction pour aller tout droit
  Rotation180(1,2000);
  Avancer(40,2000); //Appel de la fonction pour aller tout droit
  Rotation180(3,2000);
  //RadiusTurn(25,360,0,200);
  //delay(5000);*/

/*/
  RadiusTurn(25,239,0,0);
  Avancer(87.02,0); //Appel de la fonction pour aller tout droit
  RadiusTurn(25,239,1,0);
  Avancer(87.02,0); //Appel de la fonction pour aller tout droit
*/

  
  // put your main code here, to run repeatedly:
  Avancer( 113,0); //Appel de la fonction pour aller tout droit
  Tourner ( 1, 90,100); // Appel de la fonction pour tourner à gauche 900
  Avancer( 72 ,100 ); //Appel de la fonction pour aller tout droit
  Tourner (0, 90,100 ); // Appel de la fonction pour tourner à droite 900
  Avancer(83.8,100); //Appel de la fonction pour aller tout droit
  Tourner (0, 45,100 ); // Appel de la fonction pour tourner à droite 450
  Avancer(168.5,100); //Appel de la fonction pour aller tout droit
  Tourner ( 1,90,100); // Appel de la fonction pour tourner à gauche 900
  Avancer(50,100); //Appel de la fonction pour aller tout droit
  Tourner (0, 45,100); // Appel de la fonction pour tourner à droite 450
  Avancer (115,100); //Appel de la fonction pour aller tout droit
  Rotation180(1,100);
  Avancer (115,100);
  Tourner (1, 45,100);
  Avancer(50,100);
  Tourner ( 0,90,100);
  Avancer(171.5,100);
  Tourner (1, 45,100 );
  Avancer(83.8,100);
  Tourner (1, 90,100 );
  Avancer( 72 ,100 );
  Tourner ( 0, 90,100);
  Avancer( 120,0);
  Rotation180(9,100);
  
  
}

/*
==========================

==========================
*/

 void Avancer(float Distance,int TempsAttente)
 {
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1); 
    float NbPulse=(3200*Distance)/(23.9389);

   // MOTOR_SetSpeed(0, 0.1);
    //MOTOR_SetSpeed(1, 0.1);
    MOTOR_SetSpeed(0, speed);
    MOTOR_SetSpeed(1, speed);
    int PulseCount=ENCODER_Read(0);
    long Previous_time = millis();
    float ActualSpeed = speed;
    while(PulseCount<=NbPulse)
    {
      /*if (300<=(millis()-Previous_time) && ActualSpeed<speed)// 10cm = 1330 pulse
      {
          ActualSpeed=ActualSpeed*1.25;
      }*/
      PulseCount=ENCODER_Read(0);
      if (PulseCount>=NbPulse-3000 && dT<=(millis()-Previous_time))// 10cm = 1330 pulse
      {
        ActualSpeed=ActualSpeed*0.90;
      }
      MOTOR_SetSpeed(0, ActualSpeed);
      Previous_time=PID(Previous_time,ActualSpeed);
    }
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
  
    delay(TempsAttente);
  
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1);
  }
/*
==========================

==========================
*/
 void Tourner(int Direction, int Rotation,int TempsAttente)
 {
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  float NbPulse=((2*3200*3.14*L*Rotation)/(23.9389*360));//(2*PI*18.350(Distance entre roue)*3200*Rotation)/(360*23.9389(Nb cm par tour de roue))
  
  MOTOR_SetSpeed(Direction, speed);
  int PulseCount = ENCODER_Read(Direction);
  while(PulseCount<=NbPulse)
  {
      PulseCount=ENCODER_Read(Direction);
  }
  MOTOR_SetSpeed(Direction, 0);
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  delay(TempsAttente);
 }
 /*
==========================

==========================
*/
void Rotation180 (int NbRot, int TempsAttente)
{
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  float NbPulse=((3200*3.14*L*NbRot*0.9725)/(2*23.9389));//.972=correction robot jean-claude
  MOTOR_SetSpeed(0, speed);
  MOTOR_SetSpeed(1,-speed);
  int PulseCount=ENCODER_Read(0);
  while(PulseCount<=NbPulse)
  {
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

==========================
*/

void RadiusTurn(float Radius,int angle,int Direction  ,int TempsAttente)
{

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  float NbPulse=((3200*2*3.1416*Radius*angle*0.95)/(360*23.9389));
  int PulseCount;
  MOTOR_SetSpeed(Direction, speed);
  if (Direction==0)
  {
    MOTOR_SetSpeed(1,(speed*(Radius-L)/Radius));
  }
  else
  {
    MOTOR_SetSpeed(0,(speed*(Radius-L)/Radius));
  }
  PulseCount=ENCODER_Read(Direction);
 // long Previous_time = millis();
  while(PulseCount<=NbPulse)
  {
      PulseCount=ENCODER_Read(Direction);
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

 long PID(long PreviousTime,float TargetSpeed)
 {
  long CurrentTime = millis();
  unsigned int TimeSample = CurrentTime-PreviousTime;

  if (TimeSample>=dT)
  {

    int encodeur_0 = ENCODER_Read(0);
    int encodeur_1 = ENCODER_Read(1);
    Erreur = encodeur_0 - encodeur_1;
    Serial.println(Erreur);
    pid = (Erreur * Kp) + ((Erreur*Ki)/sec) + TargetSpeed;
    Serial.println(pid); 
    PreviousTime=CurrentTime;
    MOTOR_SetSpeed(1,pid);
 
  } 
  return PreviousTime;
 }