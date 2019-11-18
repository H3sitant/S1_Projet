#include <Arduino.h>
#include <LibRobus.h>
#include<math.h>

#include <stdlib.h>
#include "Adafruit_TCS34725.h"

#include "partition_1.txt"
#include "partition_2.txt"
#include "partition_3.txt"

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
#define Couleur Jaune
#define BPM 80 000



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

//traitement midi
int traitement_fichier (FILE* partition);

//Initialisation des variables globales
float speed=0.2;
float L=18.350;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
int incomingByte = 0;

void setup() 
{
  // put your setup code here, to run once:
  	Serial.begin(9600); 
    Serial1.begin(31250); 
    Serial.println("test2");
    Serial.println(Serial1.available());
    Serial.println(Serial1.read()); 
    delay(1000);
    BoardInit();
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{
  FILE* partition = fopen("parition_1.txt","r");
  //traitement_fichier(partition);
  fclose(partition);
  /*unsigned int start_time = millis();
  Serial.println(millis()-start_time);
  delay(5000);
  Serial.println(millis()-start_time);
  */
  /*if (Serial1.available() > 0)
  {
    // read the incoming byte:
    incomingByte = Serial1.read();

    // say what you got:
    if(incomingByte==40)
    {
      Serial.print("I received: ");
      Serial.println(incomingByte, DEC);
      
    }
  }*/
  
}

/*
====================================
TRAITEMENT DES COUPS
====================================
 */
/*int traitement_fichier (FILE* partition)
{
  int tempo[20];
  int instrument[20];
  int i=0;
  while(fgetc(partition)!=EOF)
  {
    fscanf(partition, "%d  %d", &tempo[i], &instrument[i]);
    i++;
    Serial.print(i);
  }
  return 1;
}*/