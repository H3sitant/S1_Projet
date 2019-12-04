#include <Arduino.h>
#include <LibRobus.h>
#include<math.h>
#include "Adafruit_TCS34725.h"

//traitement midi
int comparaison_midi(int instrument, int temps, int partition_C[2][70], int Note[2][70]);
void choix_partition (int partition_C[2][70], int partition_I[2][70]);
int music( int partition_C[2][70],int niveau);
void resultat(int note);
void tempo(void);
int TransmissionSansFil(int ValeurTx);
int ReceptionSansFil(int ValeurAttendu);
int TransmissionAvecFil(int ValeurTx);
int ReceptionAvecFil(int ValeurAttendu);
void(*resetFunc)(void)=0;

//communication
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

//Niveau de difficulté
#define NiveauFacile 10
#define NiveauMoyen 20
#define NiveauDifficile 30

#define RobotMaestroPret 35
#define SignalDepart 40

RF24 radio(53,48);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xE7E7E7E7E7LL, 0xF0F0F0F0D2LL };
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

//// PIN /////
#define BleuD 43
#define RougeD 44
#define VertD 45
#define Metronome A6

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
    pinMode(8,INPUT);
    pinMode(9,INPUT);
    pinMode(10,INPUT);
    pinMode(11,INPUT);
    pinMode(12,INPUT);
    pinMode(47,OUTPUT);
    pinMode(48,INPUT);
    pinMode(BleuD, OUTPUT);
    pinMode(RougeD, OUTPUT); 
    pinMode(VertD, OUTPUT);
     printf_begin();
  
  radio.begin();
  radio.setChannel(118);
  radio.setAutoAck(false);
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();
  radio.printDetails();
}

/*
    Appel des fonctions pour réaliser le parcours
 */
void loop() 
{
 
  // optionally, increase the delay between retries & # of retries
  /*radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.startListening();*/
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

  // First, stop listening so we can talk.
  //radio.stopListening();

  // Take the time, and send it.  This will block until complete
    
  if(ROBUS_IsBumper(1)==true)
  { 
    int i=0;
    while(i==0)
    {
      int ana = analogRead(Metronome);
      if(ana>550)
      Serial.println(ana);
    }

  }
  int partition_C[2][70];
  if(digitalRead(8)==HIGH)//choisir partition facile : changer pour Bouton Vert
  {
    radio.stopListening();
    //TransmissionSansFil(NiveauFacile);
    Serial.println("Facile");
    choix_partition ( partition_C, partition_F);
    
    int retour= music( partition_C,NiveauFacile); 
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else  resultat(retour);
    Serial.println("FIN");
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
  }
  if(digitalRead(9)==HIGH)//choisir partition Moyenne : changer pour Bouton Vert
  {
    radio.stopListening();
    //Envoyer moyen
    //TransmissionSansFil(NiveauMoyen);
    
    Serial.println("Moyen");
    choix_partition ( partition_C, partition_M);
    int retour= music( partition_C,NiveauMoyen);
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else  resultat(retour);
    Serial.println("FIN");
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
  }
  if(digitalRead(10)==HIGH)//choisir partition Difficile : changer pour Bouton Vert
  {
    radio.stopListening();
    //Envoyer difficile
    //TransmissionSansFil(NiveauDifficile);

    Serial.println("Difficile");
    choix_partition ( partition_C, partition_D);
    int retour= music( partition_C,NiveauDifficile);
    if (retour==Erreur)
    {
      Serial.println("ERREUR Temps");
      delay(10000);//stall
    }else  resultat(retour);
    Serial.println("FIN");
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
  }
}

/*
============================
Fonction coups
============================
*/
int music( int partition_C[2][70],int niveau)
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
  //if(niveau==NiveauFacile)delay(9000);
  //else if(niveau==NiveauMoyen)delay(13000);
  //else delay(17000);
  
  //ReceptionSansFil(RobotMaestroPret);
  //while(ROBUS_IsBumper(2)!=true);
  while(analogRead(Metronome)<500);
  
  //unsigned int temp=millis();
  //TransmissionSansFil(SignalDepart);
  //while(((millis()-temp)%1000)!=0);
  tempo();

  unsigned long Temps_I=0;
  Temps_I=millis();
  int note=100;
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
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);

  delay(985);
  digitalWrite(BleuD,1);
  digitalWrite(RougeD,1);
  digitalWrite(VertD,1);
  delay(15);
  digitalWrite(BleuD,0);
  digitalWrite(RougeD,0);
  digitalWrite(VertD,0);
  delay(1000);
}

int TransmissionSansFil(int ValeurTx)
{
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  
  // First, stop listening so we can talk.
  radio.stopListening();

  byte message = ValeurTx;
  printf("Now sending  ");
  Serial.print(message);
  bool ok = 0;
    ok = radio.write( &message, sizeof(message) );

 do{     
    if (ok){
      printf("Envoyé \n");
    return 1;
    }
    else{
      printf("Erreur. \n\r");
      delay(1000);
    }
  } while (ok!=true);
  radio.startListening();
  return 0;
}


int ReceptionSansFil(int ValeurAttendu)
{
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  // if there is data ready
  radio.startListening();

  byte message=0;
    do
    {
      delay(100);
      if ( radio.available() )
      {
        // Dump the payloads until we've gotten everything
        
        bool done = false;
        while (!done)
        {
          // Fetch the payload, and see if this was the last one.
          done = radio.read( &message, sizeof(message) );

          // Spew it
          printf("Got payload:  ");
          Serial.println(message);

        radio.stopListening();
        } 
      }
    } while(message !=ValeurAttendu);
  return message;
}


int TransmissionAvecFil(int ValeurTx)
{
  Serial1.write(ValeurTx);
  return 1;

}


int ReceptionAvecFil(int ValeurAttendu)
{
  int ValeurRx;
  do{
    if (Serial1.available()) {
      ValeurRx = Serial1.read();
    }
    printf("Got payload:  ");
    Serial.println(ValeurRx);
  }while(ValeurRx != ValeurAttendu);

  return ValeurRx;
}


