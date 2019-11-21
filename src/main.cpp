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

//traitement midi
int comparaison_midi(int instrument, int temps, int partition_C[2][70]);
void choix_partition (int partition_C[2][70], int partition_I[2][70]);
int music( int partition_C[2][70]);

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
//difficulté
#define facile 1
#define moyen 2
#define Difficile 3
//résultat
#define reussie 1
#define mauvaise_note 2
#define manquer 3
#define Erreur 401

int i=0;
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
    Serial2.begin(31250); 
    Serial.println("test2");
    delay(1000);
    BoardInit();
    start_time = millis();
    pinMode(22,OUTPUT);
    pinMode(23,OUTPUT);
    pinMode(24,OUTPUT);
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{ 
  int ana =analogRead(A4);
  
  
  
  /*if(ana>30)
  {
    Serial.println(i);
    digitalWrite(23,HIGH);
    delay(15);
    digitalWrite(23,LOW);
    delay(25);
    i++;
  }*/
   if(ana>30)
  {
    for (int i=0;i<=50; i++)
      {
        digitalWrite(22,HIGH);
        delay(15);
        digitalWrite(22,LOW);
        delay(25);
        digitalWrite(23,HIGH);
        delay(15);
        digitalWrite(23,LOW);
        delay(25);
        digitalWrite(24,HIGH);
        delay(15);
        digitalWrite(24,LOW);
        delay(25);
      }
  }
  /*
  choix de partition avec bouton
  */
  int partition_C[2][70];
  if(ROBUS_IsBumper(0)==true)//choisir partition facile : changer pour Bouton Vert
  {
    choix_partition ( partition_C, partition_F);
    int retour= music( partition_C);
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else
    {
      Serial.println(retour);
    }
    Serial.println("FIN");
  }
  if(ROBUS_IsBumper(1)==true)//choisir partition Moyenne : changer pour Bouton Vert
  {
    choix_partition ( partition_C, partition_M);
    Serial.println("Moyen");
    int retour= music( partition_C);
    if (retour==Erreur)
    {
      Serial.println("ERREUR DIMENTION");
      delay(10000);//stall
    }else
    {
      Serial.println(retour);
    }
    Serial.println("FIN");
  }
  if(ROBUS_IsBumper(2)==true)//choisir partition Difficile : changer pour Bouton Vert
  {
    choix_partition ( partition_C, partition_D);
    Serial.println("Difficile");
    int retour= music( partition_C);
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else if (retour==100)
    {
      Serial.println( "resultat: PARFAIT");
      for (int i=0;i<=50; i++)
      {
        digitalWrite(22,HIGH);
        delay(15);
        digitalWrite(22,LOW);
        delay(25);
        digitalWrite(23,HIGH);
        delay(15);
        digitalWrite(23,LOW);
        delay(25);
        digitalWrite(24,HIGH);
        delay(15);
        digitalWrite(24,LOW);
        delay(25);
      }
    }
    else if (retour>75)
    {
      Serial.println( "resultat: BON");
      digitalWrite(24,HIGH);
      delay(5000);
      digitalWrite(24,LOW);
    }
    else if (retour>0)
    {
      Serial.println( "resultat: MOYEN");
      digitalWrite(23,HIGH);
      delay(5000);
      digitalWrite(23,LOW);
    }else 
    {
      Serial.println( "resultat: OUCH");
      digitalWrite(22,HIGH);
      delay(5000);
      digitalWrite(22,LOW);
    }
    Serial.println("FIN");
  }
}

/*
============================
Fonction coups
============================
*/
int music( int partition_C[2][70])
{
  int Temps_I=millis();
  int note=100;
  while (millis()-Temps_I<=16125)
  {
    int Entre1 =analogRead(A4);
    int Entre2 =analogRead(A5);
    if( Entre1>30 ) //|| Entre2
    {
      
      int retour;
      if (Entre1>30)
      {
        retour=comparaison_midi(0,millis()-Temps_I, partition_C);
      }else
      {
        retour=comparaison_midi(1,millis()-Temps_I, partition_C);
      }

      if (retour==Erreur)
      {
        return Erreur;
      }else if(retour==manquer) // coups pas sur une note
      {
        digitalWrite(22,HIGH);
        delay(40);
        digitalWrite(22,LOW);
        note-=4;
        Serial.println("I received: manquer");
      }else if(retour==mauvaise_note) // mauvaise note
      {
        digitalWrite(23,HIGH);
        delay(40);
        digitalWrite(23,LOW);
        note-=2;
        Serial.println("I received: mauvaise note");
      }else if(retour == reussie)//coups réussie
      {
        digitalWrite(24,HIGH);
        delay(40);
        digitalWrite(23,LOW);
        Serial.println("I received: Reussi");
      }else
      {
        Serial.println("I received: Impossible");
      }
    }
  } 
  return note;
}

/*
====================================
Comparaison midi
====================================
 */
int comparaison_midi (int instrument, int temps, int partition_C[2][70])
{

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
  if (positionT>=70)
  {
    return Erreur;
  }
  else// comparaison du coups avec le fichier midi
  {

    if(partition_C[0][positionT]==0 && partition_C[1][positionT]==0)// pas de coups a se moment
    {
      return manquer;
    }else if( partition_C[instrument][positionT]==1)// il ya un coups a se moment et ces le bon instrument
    {
      return reussie;
    }else if( partition_C[abs(instrument-1)][positionT]==1)//il a un coups a se moment mais ce n'est pas le bon instrument
    {
      return mauvaise_note;
    }else
    {
      return 4;
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