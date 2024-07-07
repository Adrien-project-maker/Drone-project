/*
InDev Controller Arduino R3 .V0
05/07/2024
*/

// Appel des differentes librairies
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// Variables associées à des broches numériques
const int SW_pin = 2; 
const int X_pin_gaz = A0; 
const int Y_pin_gaz = A1; 
const int X_pin_controle = A2;
const int Y_pin_controle = A3;

// Broches de l'antenne NRF24L01
#define pinCE     7         
#define pinCSN    8         

// Intervalle de puissance de fonctionnements des moteurs
#define ImpulsionMin     1000        
#define ImpulsionMax     2000        

// On donne le nom "PIPE1" au tunnel de communication utilisé
#define tunnel  "PIPE1"       

// Instanciation du NRF24L01
RF24 radio(pinCE, pinCSN);  

// Mise au format "byte array" du nom du tunnel
const byte adresse[6] = tunnel;               

// Variables de lectures des joysticks
int LectureGaz;           
int LectureRoll;
int LecturePitch;
int LectureYaw;

// Puissances moteurs et commandes directionnelles par défaut
int ImpulsionGaz = 1000;   
int ImpulsionRoll = 0;
int ImpulsionPitch = 0;
int ImpulsionYaw = 0;

void setup() {

// Demarrage moniteur Serie
Serial.begin(9600);
Serial.println("Go");

// Demarrage radio
radio.begin();                           
radio.openWritingPipe(adresse);     
radio.stopListening();
radio.setChannel(124);
radio.setPALevel(RF24_PA_HIGH);
radio.setDataRate(RF24_1MBPS);

delay(1000);             
}

void loop() {

// Lecture de l'inclinaison des joysticks
LectureGaz = analogRead(X_pin_gaz);
LectureYaw = analogRead(Y_pin_gaz);
LecturePitch = analogRead(X_pin_controle);
LectureRoll = analogRead(Y_pin_controle);

// On augmente la puissance des gaz si le joystick est vers le haut
if (LectureGaz >= 530){
  ImpulsionGaz = ImpulsionGaz - (LectureGaz/1023) *10;
}

// On diminue la puissance des gaz si le joystick est vers le bas
else if (LectureGaz <= 490){
  LectureGaz = map(LectureGaz, 490, 0, 530, 1023);
  ImpulsionGaz = ImpulsionGaz + (LectureGaz/1023) *10;
}

// La puissance reste constante si le joystick est au repos
else if (LectureGaz < 530 and LectureGaz > 490){
  ImpulsionGaz = ImpulsionGaz;
}

// Même opération pour les commandes de contrôle sauf que les valeurs sont dyanmiques
// Pour le Roll
if (LectureRoll >= 530){
  ImpulsionRoll = map(LectureRoll, 530, 1023, 1500, ImpulsionMin);
}
else if (LectureRoll <= 490){
  ImpulsionRoll = map(LectureRoll, 490, 0, 1500, ImpulsionMax);
}
else if (LectureRoll < 530 and LectureRoll > 490){
  ImpulsionRoll = 1500;
}

// Pour le Pitch
if (LecturePitch >= 530){
  ImpulsionPitch = map(LecturePitch, 530, 1023, 1500, ImpulsionMin);
}
else if (LecturePitch <= 490){
  ImpulsionPitch = map(LecturePitch, 490, 0, 1500, ImpulsionMax);
}
else if (LecturePitch < 530 and LecturePitch > 490){
  ImpulsionPitch = 1500;
} 

// Pour le Yaw
if (LectureYaw >= 530){
  ImpulsionYaw = map(LectureYaw, 530, 1023, 1500, ImpulsionMin);
}
else if (LectureYaw <= 490){
  ImpulsionYaw = map(LectureYaw, 490, 0, 1500, ImpulsionMax);
}
else if (LectureYaw < 530 and LectureYaw > 490){
  ImpulsionYaw = 1500;
}

// On envoi la valeur d'une variable par la radio
radio.write(&ImpulsionGaz,  sizeof(ImpulsionGaz));

// On reboucle après un court delai
delay(20);  
}
