#include <Arduino.h>
#include <LibRobus.h>

#include <stdlib.h>
#include "Adafruit_TCS34725.h"

#define Bleu 1
#define Vert 2
#define Rouge 3
#define Jaune 4

#define ligne 1
#define Millieux 2
#define Vide 0

/*
  Couleur où se trouve le ballon
 */
#define Couleur Bleu


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
/*
======================
Code Robot A
======================
 */
if(ROBUS_IsBumper(3)==true)
{
  delay(1000);
  //180 pour aller vers la ligne
  //Rotation(180,500);
  // reculer pour trouver la ligne
  do
  {
    Avancer(1,0,0.1,0);
  }while (detecteurLigne(speed,0)== Vide);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);

  // Tourner pour etre parallèle a la ligne
  Tourner(0,90,1000,1,speed);

  //Vérifier la detection de la ligne
  if (detecteurLigne(speed,0)!=1)
  {
      delay(5000);
  }

  do
  {
    detecteurLigne(speed,1);
  }while (detectionCouleur()==0);

  while (detectionCouleur()!=Couleur)
  {
    Avancer(15.23,500,speed,1);
    Tourner(0,90,500,speed,1);
    Avancer(50, 500,speed,1);//28.73+ distance roue capteur
    do
    {
      detecteurLigne(speed,1);
    }while (detectionCouleur()==0);
  }
}


/*
======================
Code de test fonction
======================
 */
/*  if(ROBUS_IsBumper(3)==true)
  {
    Serial.println("bumper arriere");
       SERVO_Enable(0);
        SERVO_SetAngle(0, 20);
        delay(500);
        SERVO_SetAngle(0, 150);
        delay(2000);
        SERVO_SetAngle(0, 20);
        delay(500);
        SERVO_Disable(0);
        int i=0;
    while (i<=65000)
    {
      detecteurLigne(0.3,150,150,7);
      i++;
    }
    Serial.println("fin");
    delay(500);
    
  }
   

  if(detectionCouleur() == Couleur)
  {
    Avancer(5,1000);
    servomoteurPrendre();
    Rotation(180,100);
    //Tourner(0,180,0);
    Avancer(112.08,0);
    servomoteurLacher();
    Tourner(0,135,0);
    Avancer(60,0);
  }

  while (detectionCouleur() != Couleur)
  {
    Rotation(135,100);
     //Tourner(0,135,0);
     Avancer(151.4,0);
  }
  
  if(detectionCouleur() == Couleur)
  {
    Avancer(5,1000);
    servomoteurPrendre();
    Tourner(0,180,0);
    Avancer(112.8,0);
    servomoteurLacher();
    Tourner(0,135,0);
    Avancer(60,0);
  }
  }

  

    while (detectionCouleur() != Couleur)
    {
      detecteurLigne(0);
    }
    //Avancer(-28,0);
   // delay(1000);

    while (detecteurLigne(0)==0)
    {
      Tourner(1,5,0);
    }
    delay(1000);
    while (detectionCouleur() != Couleur)
    {
       while (detectionCouleur() == 0)
     {
        Avancer(1,0);
        detecteurLigne(speed);
      }
      delay(1000);
      if(detectionCouleur() != Couleur)
      {
        Avancer(15.23, 500);
        Tourner(0,90,500);
        Avancer(50, 500);//28.73+ distance roue capteur
      }
    }
    TrouverBallon();
    Rotation180(1,500);
    while (detecteurLigne(0)!=5)
    {
      Avancer(1,0);
      detecteurLigne(speed);
    }
    servomoteurLacher();

    //aller position pas dérengente
  }
*/
}

/*==========================
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
    unsigned int PulseCount=ENCODER_Read(0);
    long Previous_time = millis();
    long CurrentTime = millis();
    unsigned int TimeSample = 0;

   while(PulseCount<=NbPulse)
    {
      Serial.println(PulseCount);   
      TimeSample = CurrentTime-Previous_time;
      PulseCount=ENCODER_Read(0);
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
    Serial.println("Fin avancer");
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
  unsigned int PulseCount = ENCODER_Read(Direction);
  Serial.println(NbPulse);
  while(PulseCount<=NbPulse)
  {    
    MOTOR_SetSpeed(Direction,ActualSpeed);
    PulseCount=ENCODER_Read(Direction);
    Serial.println(PulseCount);
  }
  if (arret==1){
  MOTOR_SetSpeed(Direction, 0);}

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  delay(TempsAttente);
  Serial.println("Fin de Tourner");
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
  Serial.println(NbPulse);
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
  SERVO_SetAngle(0, 20);
  SERVO_Enable(0);
}

/*
==========================
Servomoteurs lacher
==========================
 */

void servomoteurLacher ()
{
  SERVO_Enable(0);
  SERVO_SetAngle(0, 150);
  SERVO_Disable(0);
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
  else if(tension < 1) //option 2(X3) 0.71
  {
    Detect=ligne;
  }
  else if(tension< 1.9) //option 3(X2) 1.51
  {
    Detect=ligne;
  }
  else if(tension< 2.40) //option 4(X2,X3) 2.14
  {
    if (type==1)
    {
      MOTOR_SetSpeed(1,0);
      MOTOR_SetSpeed(0,0);
      Tourner(1,5,0,0,-TargetSpeed*1.7);
    }
    Detect=ligne;
  }
  else if(tension< 3) //option 5(X1) 2.85
  {
    Detect=ligne;
  }
  else if(tension< 3.9) //option 6(X1,X3) 3.57
  {
   
   Detect=ligne;
  }
  else if(tension< 4.5) //option 7(X1,X2) 4.28
  {
    if (type==1)
    {
      MOTOR_SetSpeed(1,0);
      MOTOR_SetSpeed(0,0);
      Tourner(0,5,0,0,-TargetSpeed*1.7);
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
	 uint16_t distance=ROBUS_ReadIR(1);    // reads the value of the sharp sensor
	 Serial.println(distance);            // prints the value of the sensor to the serial monitor
	 delay(200);
   //return;
                       // wait for this much time before printing next value

    //Les valeurs pour la longueure des rayons sont inconnue pour l'instant. Nous allons utiliser le therme: Longueur_des_rayons_IR
  int Longeur_des_rayons_IR =0;
  int ballon_trouve ;
  if (ballon_trouve == Longeur_des_rayons_IR)
    {
      /*
      ==========================
      Servomoteurs prendre
      ==========================
      */
      void servomoteurPrendre ();
    }             // wait for this much time before printing next value
 }