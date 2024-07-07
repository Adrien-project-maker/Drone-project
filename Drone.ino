/*
InDev Drone ESP 32 WROOM 32 .V0
07/07/2024
*/

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


// On associe les moteurs à des broches capables d'emettre une fréquence
#define Moteur_avant_droit       14
#define Moteur_avant_gauche      27

#define Moteur_arriere_droit           12
#define Moteur_arriere_gauche          26

// Broches de l'antenne NRF24L01
#define pinCE     0
#define pinCSN    4

// Intervalle de puissance de fonctionnements des moteurs
#define ImpulsionMin     1000
#define ImpulsionMax     2000

// On donne le nom "PIPE1" au tunnel de communication utilisé
#define tunnel  "PIPE1"

// On assigne un identifiant unique aux capteurs
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);


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
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
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

// Puissance des moteurs à l'arret
int ThrottleCutOff=1000;

// On Vérifie que l'on a accès aux différents capteurs
void initSensors()
{
  if(!accel.begin()){
    // On envoi un message dans le moniteur serie si on ne détecte pas le capteur
    Serial.println(F("Pas de LSM303 détecté, vérifiez vos cablages !"));
    while(1);
  }
  if(!mag.begin()){
    // On envoi un message dans le moniteur serie si on ne détecte pas le capteur
    Serial.println("Pas de LSM303 détecté, vérifiez vos cablages !");
    while(1);
  }
  if(!bmp.begin()){
    // On envoi un message dans le moniteur serie si on ne détecte pas le capteur
    Serial.println("Pas de BMP180 détecté, vérifiez vos cablages !");
    while(1);
  }
}

// On récupère dans l'orientation du drone sur l'axe X Y Z dans cette fonction
void gyro_signals(void) {

  // On donne un identifiant à chaque capteurs
  sensors_vec_t   orientation;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;

  // Si on a accès à l'orientation on la stocke dans une variable
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)){
    int GyroX = orientation.roll;
    int GyroY = orientation.pitch;

    RateRoll=(float)GyroX/65.5;
    RatePitch=(float)GyroY/65.5;

    Serial.print("GyroX ");
    Serial.print(GyroX);
    Serial.println("");
    Serial.print("GyroY ");
    Serial.print(GyroY);
    Serial.println("");
  }

  // Même chose pour le Yaw
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)){
    int GyroZ = orientation.heading;

    RateYaw=(float)GyroZ/65.5;
    
    Serial.print("GyroZ ");
    Serial.print(GyroZ);
    Serial.println("");
  }
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
  initSensors();
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

  // On récupere l'orientation initiale du drone
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    // On fait appel à la fonction qui lit les valeurs du gyroscope
    gyro_signals();
    // On y opère les premières opération pour convertir l'orientation dans un ordre de grandeur compatible
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
    RateCalibrationRoll/=2000;
    RateCalibrationPitch/=2000;
    RateCalibrationYaw/=2000;
  }
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
      RateRoll-=RateCalibrationRoll;
      RatePitch-=RateCalibrationPitch;
      RateYaw-=RateCalibrationYaw;
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
      int ThrottleIdle=1180;
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
      delay(20);
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