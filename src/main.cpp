//Jean-Claude
#include <Arduino.h>
#include <LibRobus.h>

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

void loop() {
/*
Direction:  0 = Rotation Horraire     1 = Rotation AntiHorraire
*/

  /*Avancer(100,2000); //Appel de la fonction pour aller tout droit
  Rotation180(1,2000);
  Avancer(100,2000); //Appel de la fonction pour aller tout droit
  Rotation180(3,2000);*/
  //RadiusTurn(25,360,0,200);
  //delay(5000);*/

  
  //Avancer(308.4,5000); //Appel de la fonction pour aller tout droit

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
  Avancer(171.5,100); //Appel de la fonction pour aller tout droit
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
  Rotation180(1,100);
  Rotation180(1,100);
  Rotation180(1,100);
  
  
}

/*
==========================

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

==========================
*/

void RadiusTurn(float Radius,int angle,int Direction  ,int TempsAttente)
{

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  unsigned int NbPulse=((3200*2*3.1416*Radius*angle*0.95)/(360*23.9389));
  unsigned int PulseCount;
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