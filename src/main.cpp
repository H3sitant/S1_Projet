#include <Arduino.h>
#include <LibRobus.h>
#include<math.h>
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
int comparaison_midi(int instrument, int temps, int partition_C[2][70]);
void choix_partition (int partition_C[2][70], int partition_I[2][70]);

/*
====================
traitement midi
====================
*/
//partition
int partition_F[2][70] =  {//facile
                          { 0,0,0,0 ,0,0,0,0 ,0,0,0,0 ,0,0,0,0,  1,0,1,0, 1,0,1,0, 1,0,1,0 ,1,0,1,0,  0,0,0,0, 1,0,0,0 ,0,0,0,0, 1,0,0,0,  0,0,0,0 ,1,0,0,0 ,0,0,0,0, 1,0,0,0,  1,0,0,0, 0,0},
                          { 1,0,1,0 ,1,0,1,0 ,1,0,1,0 ,1,0,1,0,  0,0,0,0 ,0,0,0,0 ,0,0,0,0 ,0,0,0,0,  1,0,1,0 ,1,0,1,0 ,1,0,1,0 ,1,0,1,0,  1,0,1,0 ,1,0,1,0 ,1,0,1,0 ,1,0,1,0,  1,0,0,0, 0,0}
                          };
int partition_M[2][70] =  {//moyenne
                          { 1,0,0,0 ,0,0,0,0 ,1,0,0,0 ,1,0,0,0,  0,0,0,0 ,1,0,0,0 ,0,0,0,0 ,0,0,0,0,  1,0,0,0 ,1,0,1,0 ,0,0,1,0 ,0,0,0,0,  1,0,1,1 ,0,1,0,0 ,1,0,1,1 ,0,1,0,0,  1,0,0,0, 0,0},
                          { 0,0,0,0 ,1,0,0,0 ,0,0,0,0 ,0,0,0,0,  1,0,0,0 ,0,0,0,0 ,1,0,0,0 ,1,0,0,0,  0,0,1,0 ,0,0,0,0 ,1,0,0,0 ,1,0,1,0,  0,1,0,0 ,1,0,1,1 ,0,1,0,0 ,1,0,1,1,  0,0,0,0, 0,0}
                          };
int partition_D[2][70] =  {//difficile
                          { 0,0,0,0 ,1,0,0,0 ,0,0,0,0 ,1,0,0,0,  0,0,1,0 ,0,0,1,0 ,0,0,1,0 ,0,0,1,0,  0,0,1,0 ,0,0,1,0 ,0,1,0,0 ,1,1,1,1,  1,0,0,0 ,0,0,0,0 ,0,0,0,0 ,0,0,0,0, 0,0,0,0 ,0,0},
                          { 1,0,1,0 ,0,0,1,0 ,0,0,1,0 ,0,0,1,0,  1,1,0,0 ,1,1,0,0 ,1,1,0,0 ,1,1,0,0,  1,1,0,1 ,1,0,0,0 ,1,0,1,1 ,0,0,0,0,  1,0,0,0 ,0,0,0,0 ,0,0,0,0 ,0,0,0,0, 0,0,0,0 ,0,0}
                          };
//temps
unsigned int start_time;
//instrument
#define instrument1 40
#define instrument2 30
//dificulté
#define facile 1
#define moyen 2
#define Difficile 3
//résultat
#define reussie 1
#define mauvaise_note 2
#define manquer 3
#define Erreur 0

/*
=====================

=====================
*/
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
    start_time = millis();
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{

  /*
  choix de partition avec bouton
  */
  int partition_C[2][70];
  //choix_partition ( *partition_C[2][70], partition_I[2][70]);
  //traitement_fichier(partition);
  if (millis()-start_time<=16.125)
  {
    if (Serial1.available() > 0)
    {
      // read the incoming byte:
      incomingByte = Serial1.read();

      // say what you got:
      if(incomingByte==instrument1 || incomingByte==instrument2)
      {
        Serial.print("I received: ");
        Serial.println(incomingByte, DEC);
        Serial.println(millis()-start_time);
        int retour;
        if (incomingByte==instrument1)
        {
          retour=comparaison_midi(instrument1,millis()-start_time, partition_C);
        }else
        {
          retour=comparaison_midi(instrument2,millis()-start_time, partition_C);
        }
        
        if (retour==Erreur)
        {
          Serial.println("ERREUR DIMENTION");
        }else if(retour==manquer) // coups pas sur une note
        {
          //Alumer lumière rouge
        }else if(retour==mauvaise_note) // mauvaise note
        {
          //Alumer lumière Jaune
        }else //coups réussie
        {
          //Alumer lumière Verte
        }
      }
    }
  } 
}

/*
====================================
Comparaison midi
====================================
 */
int comparaison_midi (int instrument, int temps, int partition_C[2][70])
{
  //trouver l'instrument
  if(instrument==instrument1)
  {
    instrument=0; //position dans tableau
  }else
  {
    instrument=1;//position dans tableau
  }

  //trouver le reste pour pouvoir identifier la position du coups dans le temps
  int positionT = temps % 250;

  if (positionT<=125)
  {
    positionT= (temps-positionT)/250; //temps du coups arondi a la baisse
  }else
  {
    positionT= (temps-positionT)/250+1;//temps du coups aroudi a la hausse
  }
  //vérifier si les dimention son bonne
  if (positionT>=64)
  {
    return 0;
  }
  else// comparaison du coups avec le fichier midi
  {
    if(partition_C[positionT][0]==0 && partition_F[positionT][1]==0)// pas de coups a se moment
    {
      return manquer;
    }else if(partition_C[positionT][instrument]==1)// il ya un coups a se moment et ces le bon instrument
    {
      return reussie;
    }else //il a un coups a se moment mais ce n'est pas le bon instrument
    {
      return mauvaise_note;
    }
  } 
}

/*
=====================================
Choix de partition
=====================================
*/
void choix_partition (int partition_C[2][70], int partition_I[2][70])
{
  for (int i=0; i<2; i++)
  {
    for (int j=0; j<70; j++)
    {
      partition_C[i][j]=partition_I[i][j];//innitialisation de la partition choisie
    }
  }
}