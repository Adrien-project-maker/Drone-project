/*
InDev Drone ESP 32 WROOM 32 .V0.2
06/10/2024

La lecture du gyro par protocole I2c fonctionne à merveille !*/

// Appel des differentes librairies
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

// On utilise le protocole I²C
#define USE_I2C

// On associe les moteurs à des broches capables d'emettre une fréquence
#define Moteur_avant_droit       12
#define Moteur_avant_gauche      14

#define Moteur_arriere_droit           26
#define Moteur_arriere_gauche          27

// Broches de l'antenne NRF24L01
#define pinCE     0
#define pinCSN    4

// Intervalle de puissance de fonctionnements des moteurs
#define ImpulsionMin     1000
#define ImpulsionMax     2000

// On donne le nom "PIPE1" au tunnel de communication utilisé
#define tunnel  "PIPE1"

// Instanciation du NRF24L01
RF24 radio(pinCE, pinCSN);

// Instanciation des moteurs
Servo ESC_Moteur_avant_droit;
Servo ESC_Moteur_avant_gauche;

Servo ESC_Moteur_arriere_droit;
Servo ESC_Moteur_arriere_gauche;

// Mise au format "byte array" du nom du tunnel
const byte adresse[6] = tunnel;

// Déclaration des variables globales
int ImpulsionGaz;
int ImpulsionRoll;
int ImpulsionPitch;
int ImpulsionYaw;

// Déclarationn des variables pour l'asservissement PID (donc en anglais)
float  RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
uint32_t LoopTimer;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
// Alpha est un facteur de lissage compris entre 0 et 1 (par exemple, 0.1 pour un filtrage plus fort)
float alpha = 0.1;

// Variables pour stocker les mesures filtrées
float gyro_X_filtered = 0;
float gyro_Y_filtered = 0;
float gyro_Z_filtered = 0;

// Puissance des moteurs à l'arret
int ThrottleCutOff=1000;

uint8_t i2c_read(uint8_t device_address, uint8_t register_address) {
  Wire.beginTransmission(device_address);
  Wire.write(register_address);  // Envoi de l'adresse du registre à lire
  Wire.endTransmission(false);   // Reprise de la communication sans "stop"
  Wire.requestFrom(device_address, 1);  // Demande d'un octet

  // Attendre la réception de l'octet
  while (Wire.available() == 0);

  return Wire.read();  // Retourne l'octet lu
}

void i2c_write(uint8_t device_address, uint8_t register_address, uint8_t data) {
  Wire.beginTransmission(device_address);
  Wire.write(register_address);  // Envoi de l'adresse du registre
  Wire.write(data);              // Envoi de la donnée
  Wire.endTransmission();        // Stop de la transmission
}

void low_pass_filter(int16_t X, int16_t Y, int16_t Z) {
    gyro_X_filtered = alpha * X + (1.0 - alpha) * gyro_X_filtered;
    gyro_Y_filtered = alpha * Y + (1.0 - alpha) * gyro_Y_filtered;
    gyro_Z_filtered = alpha * Z + (1.0 - alpha) * gyro_Z_filtered;
}

// On récupère dans l'orientation du drone sur l'axe X Y Z dans cette fonction
void gyro_signals(void) {
  // Initialisation du L3GD20
  i2c_write(0x6B, 0x20, 0x0F); // Activer le gyroscope (CTRL_REG1)
  i2c_write(0x6B, 0x23, 0x20); // Configurer la pleine échelle à ±2000 dps (CTRL_REG4)
  i2c_write(0x6B, 0x20, 0x0F); // Configuration de Low Pass Filter
  /*int8_t status_reg = i2c_read(0x6B, 0x27);
  Serial.print("satus_reg :");
  Serial.print(status_reg);*/

  // Lecture des données sur X, Y, Z
  int8_t OUT_X_L = i2c_read(0x6B, 0x28);
  int8_t OUT_X_H = i2c_read(0x6B, 0x29);
  int8_t OUT_Y_L = i2c_read(0x6B, 0x2A);
  int8_t OUT_Y_H = i2c_read(0x6B, 0x2B);
  int8_t OUT_Z_L = i2c_read(0x6B, 0x2C);
  int8_t OUT_Z_H = i2c_read(0x6B, 0x2D);

  // Combiner les octets pour obtenir des valeurs 16 bits
  int16_t X = (OUT_X_H << 8) | OUT_X_L;
  int16_t Y = (OUT_Y_H << 8) | OUT_Y_L;
  int16_t Z = (OUT_Z_H << 8) | OUT_Z_L;

  // Appliquer le filtre passe-bas
  low_pass_filter(X, Y, Z);

  // Conversion en LSB/°/s (pour ±2000 dps, la sensibilité est 70 LSB/°/s)
  float Sensibility = 70.0;
  RateRoll = X / Sensibility;
  RatePitch = Y / Sensibility;
  RateYaw = Z / Sensibility;
  // !!!!!!!!!!!!!!!!!!!! Le mode de lecture des gyros à changé : adapter le code !!!!!!!!!!!!!!!!!
}

// On prépare les opérations à faire pour y faire appel plus tard dans le loop 
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm){
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

// On prévoit un reset des équations dans le cas par exemple où le drone est à l'arret
void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}


void setup() {
  // Configuration des pins de sortie de l'ESP 32
  pinMode(MotorInput1, OUTPUT);
  pinMode(MotorInput2, OUTPUT);

  pinMode(MotorInput3, OUTPUT);
  pinMode(MotorInput4, OUTPUT);

  // On associe l'objet ESC à la broche de commande de l'ESC, avec précision des durées d'impulsion Min/Max
  ESC_Moteur_avant_gauche.attach(Moteur_avant_gauche, ImpulsionMin, ImpulsionMax);
  ESC_Moteur_avant_droit.attach(Moteur_avant_droit, ImpulsionMin, ImpulsionMax);
  
  ESC_Moteur_arriere_gauche.attach(Moteur_arriere_gauche, ImpulsionMin, ImpulsionMax);
  ESC_Moteur_arriere_droit.attach(Moteur_arriere_droit, ImpulsionMin, ImpulsionMax);

  // Démarrage du moniteur serie
  Serial.begin(9600);
  Serial.println("Go");
  // Setup clock speed
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // On récupere une fois par milliseconde l'orientation initiale du drone pendant 2 secondes
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    // On fait appel à la fonction qui lit les valeurs du gyroscope
    gyro_signals();
    // On ajoute les valeurs lues aux variables de corrections
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  // On récupere la valeur moyenne sur le total des valeurs lues en 2 secondes
    RateCalibrationRoll/=2000;
    RateCalibrationPitch/=2000;
    RateCalibrationYaw/=2000;

  // Initialisation du module NRF24
  radio.begin();
  // Ouverture du tunnel en LECTURE, avec le "nom" qu'on lui a donné    
  radio.openReadingPipe(0, adresse);
  // On choisi une chaine de communication libre
  radio.setChannel(124);
  // On met la radio en mode écoute
  radio.startListening();
  // On choisi un certain débit de communications
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  // puis démarrage du programme
  delay(2000);
}

void loop() {
  // On vérifie si on reçoit des information de la radio de la manette
  if(radio.available()) {
    // On opére nos calculs tant que l'on reçoit les informations nécessaires au contrôle du drone
    while (radio.available()) {
      // On lit l'intégralité de la variable envoyée par la manette
      radio.read(&ImpulsionGaz, sizeof(ImpulsionGaz));
      int InputThrottle = ImpulsionGaz;
      // On donne une valeur arbitraire aux variables des commandes car problème de lectures de celle-ci
      int Roll = 1500;
      int Yaw = 1500;
      int Pitch = 1500;
      // Appel de la fonction qui lit l'orientation du drone
      gyro_signals();
      // Correction des veleurs lues pour avoir une orientation de 0°/s au sol
      RateRoll-=RateCalibrationRoll;
      RatePitch-=RateCalibrationPitch;
      RateYaw-=RateCalibrationYaw;

      /*Serial.print("RateRoll ");
      Serial.print(RateRoll);
      Serial.println("");
      Serial.print("RatePitch ");
      Serial.print(RatePitch);
      Serial.println("");
      Serial.print("RateYaw ");
      Serial.print(RateYaw);
      Serial.println("");*/

      // On prend en compte les commandes de la manette
      DesiredRateRoll=0.15*(Roll-1500);
      DesiredRatePitch=0.15*(Pitch-1500);
      DesiredRateYaw=0.15*(Yaw-1500);
      // On récupere l'erreur entre l'orientation réelle/cible
      ErrorRateRoll=DesiredRateRoll-RateRoll;
      ErrorRatePitch=DesiredRatePitch-RatePitch;
      ErrorRateYaw=DesiredRateYaw-RateYaw;
      // On appel les fonctions gérant les opération e l'asservissemnt PID
      pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
      // On récupere les résultats pour le Roll
      InputRoll=PIDReturn[0];
      PrevErrorRateRoll=PIDReturn[1]; 
      PrevItermRateRoll=PIDReturn[2];
      pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
      // On récupere les résultats pour le Pitch
      InputPitch=PIDReturn[0];
      PrevErrorRatePitch=PIDReturn[1]; 
      PrevItermRatePitch=PIDReturn[2];
      pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
      // On récupere les résultats pour le Yaw
      InputYaw=PIDReturn[0]; 
      PrevErrorRateYaw=PIDReturn[1]; 
      PrevItermRateYaw=PIDReturn[2];

      /* Si la puissance des moteurs est trop basse, on ne prend pas en compte les opérations de stabilisation
      afin de ne pas handicaper le drone au décollage */
      int ThrottleIdle=1345;
      if (1050 <= MotorInput1 < ThrottleIdle) MotorInput1 =  InputThrottle;
      if (1050 <= MotorInput2 < ThrottleIdle) MotorInput2 =  InputThrottle;
      if (1050 <= MotorInput3 < ThrottleIdle) MotorInput3 =  InputThrottle;
      if (1050 <= MotorInput4 < ThrottleIdle) MotorInput4 =  InputThrottle;
    
      // Si la puissance est trop basse, on coupe les moteurs
      if (InputThrottle < 1050 or InputThrottle > 5000){
        MotorInput1=ThrottleCutOff; 
        MotorInput2=ThrottleCutOff;
        MotorInput3=ThrottleCutOff; 
        MotorInput4=ThrottleCutOff;
        // On reset les opération pour quelle ne soient pas obsoletes au prochain décollage
        reset_pid();
      }

      // Limite d'impulsion
      if (InputThrottle > 2000) InputThrottle = 2000;
      // Si la puissance des moteurs est dans le bon intervalle, on prend en compte les opérations de stabilisation
      if (InputThrottle > ThrottleIdle and InputThrottle < 2000) {
        MotorInput3= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);   // penser à vérifier que l'on envoie les corrections aux bons moteurs
        MotorInput4= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
        MotorInput2= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
        MotorInput1= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);
    }

      // Enfjn, on envoi la commande de puissance à chaque moteur
      ESC_Moteur_avant_gauche.writeMicroseconds(MotorInput1);
      ESC_Moteur_avant_droit.writeMicroseconds(MotorInput2);
      ESC_Moteur_arriere_gauche.writeMicroseconds(MotorInput3);
      ESC_Moteur_arriere_droit.writeMicroseconds(MotorInput4);
      // avec une petite pause, avant de reboucler
      Serial.print("MotorInput1");
      Serial.print(MotorInput1);
      Serial.print("");
      Serial.print("MotorInput2");
      Serial.print(MotorInput2);
      Serial.print("");
      Serial.print("MotorInput3");
      Serial.print(MotorInput3);
      Serial.print("");
      Serial.print("MotorInput4");
      Serial.println(MotorInput4);
      Serial.println("");
    }
  }
  // SÉCURITÉ : ON DÉSACTIVE LES MOTEURS SI ON NE CAPTE PAS LA MANETTE
  if (!radio.available()) {
    ESC_Moteur_avant_gauche.writeMicroseconds(ThrottleCutOff);  
    ESC_Moteur_avant_droit.writeMicroseconds(ThrottleCutOff);
    ESC_Moteur_arriere_gauche.writeMicroseconds(ThrottleCutOff); 
    ESC_Moteur_arriere_droit.writeMicroseconds(ThrottleCutOff);
  }
}
// Fin du programme ... jusqu'à maintenant !
