#include <Arduino.h>
#include <LibRobus.h>
#include<math.h>
#include "Adafruit_TCS34725.h"

//traitement midi
int comparaison_midi(int instrument, int temps, int partition_C[2][70], int Note[2][70]);
void choix_partition (int partition_C[2][70], int partition_I[2][70]);
int music( int partition_C[2][70]);
void resultat(int note);
void tempo(void);

/*
====================
traitement midi
====================
*/
//partition
int partition_F[2][70] =  {//facile
                          { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0},
                          { 1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0}
                          };
int partition_M[2][70] =  {//moyenne
                          { 1,0,0,0 ,0,0,0,0 ,1,0,0,0 ,1,0,0,0,  0,0,0,0 ,1,0,0,0 ,0,0,0,0 ,0,0,0,0,  1,0,0,0 ,1,0,1,0 ,0,0,1,0 ,0,0,0,0,  1,0,1,1 ,0,1,0,0 ,1,0,1,1 ,0,1,0,0,  1,0,0,0, 0,0},
                          { 0,0,0,0 ,1,0,0,0 ,0,0,0,0 ,0,0,0,0,  1,0,0,0 ,0,0,0,0 ,1,0,0,0 ,1,0,0,0,  0,0,1,0 ,0,0,0,0 ,1,0,0,0 ,1,0,1,0,  0,1,0,0 ,1,0,1,1 ,0,1,0,0 ,1,0,1,1,  0,0,0,0, 0,0}
                          };
int partition_D[2][70] =  {//difficile
                          { 0,0,0,0 ,1,0,0,0 ,0,0,0,0 ,1,0,0,0,  0,0,1,0 ,0,0,1,0 ,0,0,1,0 ,0,0,1,0,  0,0,1,0 ,0,0,1,0 ,0,1,0,0 ,1,1,1,1,  0,0,1,0 ,0,0,1,0 ,0,0,1,0 ,0,0,1,0, 1,0,0,0 ,0,0},
                          { 1,0,1,0 ,0,0,1,0 ,0,0,1,0 ,0,0,1,0,  1,1,0,0 ,1,1,0,0 ,1,1,0,0 ,1,1,0,0,  1,1,0,1 ,1,0,0,0 ,1,0,1,1 ,0,0,0,0,  1,1,0,0 ,1,1,0,0 ,1,1,0,0 ,1,1,0,0, 1,0,0,0 ,0,0}
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

#define BleuD 43
#define RougeD 44
#define VertD 45

int randombleu=0; 
int randomrouge=0; 
int randomvert=0; 
float moyenneRNG=0;
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
    Serial.println("Begin"); 
    delay(1000);
    BoardInit();
    start_time = millis();
    pinMode(22,OUTPUT);
    pinMode(23,OUTPUT);
    pinMode(24,OUTPUT);
    pinMode(26,INPUT);
    pinMode(27,INPUT);
    pinMode(28,INPUT);
    pinMode(BleuD, OUTPUT);
    pinMode(RougeD, OUTPUT); 
    pinMode(VertD, OUTPUT);
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{ 
  int D1 =analogRead(A4);
  int D2 =analogRead(A5);
  /*if(D1>0)
  {
    Serial.print("D1:");
    Serial.println(D1);
  }
  if(D2>0)
  {
    Serial.print("D2:");
    Serial.println(D2);
  }*/
  if(D1>150||D2>100)
  {
    Serial.println(D1);
    Serial.println(D2);
    int VR=randombleu;
    int VB=randomrouge;
    int VV=randomvert;
    do 
    {
      randombleu=random(2);
      randomrouge=random(2);
      randomvert=random(2);
      moyenneRNG=randombleu+randomrouge+randomvert;
    } while (moyenneRNG == 0 || (randombleu==VB && randomrouge==VR && randomvert==VV));
    digitalWrite(BleuD,randombleu);
    digitalWrite(RougeD,randomrouge);
    digitalWrite(VertD,randomvert);
    delay(25);
  }

  /*
  choix de partition avec bouton
  */
 
  
  int partition_C[2][70];
  if(digitalRead(26)==HIGH)//choisir partition facile : changer pour Bouton Vert
  {
    Serial.println("Facile");
    choix_partition ( partition_C, partition_F);
    
    int retour= music( partition_C); 
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else  resultat(retour);
    Serial.println("FIN");
  }
  if(digitalRead(27)==HIGH)//choisir partition Moyenne : changer pour Bouton Vert
  {
    Serial.println("Moyen");
    choix_partition ( partition_C, partition_M);
    int retour= music( partition_C);
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else  resultat(retour);
    Serial.println("FIN");
  }
  if(digitalRead(28)==HIGH)//choisir partition Difficile : changer pour Bouton Vert
  {
    Serial.println("Difficile");
    choix_partition ( partition_C, partition_D);
    int retour= music( partition_C);
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else  resultat(retour);
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
   int NoteA[2][70];
  for (int i=0; i<2; i++)
  {
    for (int j=0; j<70; j++)
    {
      if(partition_C[i][j]==1)
      {
        NoteA[i][j]=0;
      }else NoteA[i][j]=1;
    }
  }
  tempo();
  unsigned long Temps_I=0;
  Temps_I=millis();
  /*Serial.print("Temps:");
  Serial.println(Temps_I);
  Serial.print("Millis:");
  Serial.println(millis());*/
  int note=100;
  
  //Serial.print("Millis-temps:");
  //Serial.println(millis()-Temps_I);
  
  while ((millis()-Temps_I)<=16125)
  {
    int Entre1 =analogRead(A4);
    int Entre2 =analogRead(A5);
    if( Entre1>150 || Entre2>70)
    {
      int retour;
      if (Entre1>150)
      {
        retour=comparaison_midi(0,millis()-Temps_I, partition_C,NoteA);
      }else
      {
        retour=comparaison_midi(1,millis()-Temps_I, partition_C,NoteA);
      }

      if (retour==Erreur)
      {
        return Erreur;
      }else if(retour==manquer) // coups pas sur une note
      {
        digitalWrite(BleuD,0);
        digitalWrite(RougeD,1);
        digitalWrite(VertD,0);
        Serial.println("I received: manquer");
      }else if(retour==mauvaise_note) // mauvaise note
      {
        digitalWrite(BleuD,1);
        digitalWrite(RougeD,0);
        digitalWrite(VertD,0);
        Serial.println("I received: mauvaise note");
      }else if(retour == reussie)//coups réussie
      {
        digitalWrite(BleuD,0);
        digitalWrite(RougeD,0);
        digitalWrite(VertD,1);
        Serial.println("I received: Reussi");
      }else
      {
        Serial.println("I received: Impossible");
      }
      delay(25);
    }
  }
  for (int i=0; i<2; i++)
  {
    for (int j=0; j<70; j++)
    {
      while(NoteA[i][j]>1)
      {
        note-=1;
        NoteA[i][j]--;
      }
      if(NoteA[i][j]==0)note-=2;
    }
  }
  Serial.println(note);
  return note;
}

/*
====================================
Comparaison midi
====================================
 */
int comparaison_midi (int instrument, int temps, int partition_C[2][70], int Note[2][70])
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
    if(partition_C[0][positionT]==1 && partition_C[1][positionT]==1)
    {
      Note[instrument][positionT]++;
      if(Note[instrument][positionT]>1||Note[abs(instrument-1)][positionT]>1)
      return manquer;
      else
      return reussie;
    }
    else if(partition_C[0][positionT]==0 && partition_C[1][positionT]==0)// pas de coups a se moment
    {
      Note[instrument][positionT]++;
      return manquer;
    }else if( partition_C[instrument][positionT]==1)// il ya un coups a se moment et ces le bon instrument
    {
      Note[instrument][positionT]++;
      if(Note[instrument][positionT]>1)
      return manquer;
      else
      return reussie;
    }else if( partition_C[abs(instrument-1)][positionT]==1)//il a un coups a se moment mais ce n'est pas le bon instrument
    {
      Note[abs(instrument-1)][positionT]++;
      if(Note[abs(instrument-1)][positionT]>1)
      return manquer;
      else
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
/*
=================================
Traitement note
=================================
*/
void resultat(int note)
{
  if (note==100)
  {
    Serial.println( "resultat: PARFAIT");
    for (int i=0;i<=100; i++)
    {
      int VR=randombleu;
      int VB=randomrouge;
      int VV=randomvert;
      int randombleu=0; 
      int randomrouge=0; 
      int randomvert=0; 
      float moyenneRNG=0;
      
      do 
      {
        randombleu=random(2);
        randomrouge=random(2);
        randomvert=random(2);
        moyenneRNG=randombleu+randomrouge+randomvert;
      } while (moyenneRNG == 0|| (randombleu==VB && randomrouge==VR && randomvert==VV));
      digitalWrite(BleuD,randombleu);
      digitalWrite(RougeD,randomrouge);
      digitalWrite(VertD,randomvert);
      delay(100);
    }
  }
  else if (note>75)
  {
    Serial.println( "resultat: BON");
    delay(100);
    digitalWrite(BleuD,0);
    digitalWrite(RougeD,0);
    digitalWrite(VertD,1);
    delay(5000);
  }
  else if (note>50)
  {
    Serial.println( "resultat: MOYEN");
    delay(100);
    digitalWrite(BleuD,1);
    digitalWrite(RougeD,0);
    digitalWrite(VertD,0);
    delay(5000);
  }else 
  {
    Serial.println( "resultat: OUCH");
    delay(100);
    digitalWrite(BleuD,0);
    digitalWrite(RougeD,1);
    digitalWrite(VertD,0);
    delay(5000);
  }  
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);
}
/*
==================
tempo
=================
*/
void tempo(void)
{
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  AX_BuzzerON(500,15);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);
  delay(1000);
}