
#include "main.h"
#include "PID.h"                  // bibliothèque PID
#include "FastInterruptEncoder.h" // bibliothèque pour les codeurs incrémentaux
#include "SimFirstOrder.h"        // bibliothèque pour la simulation du moteur
#include "Communication.h"
#include "interruption.h"
#include "BoucleOuverte.h"
#include "Odometrie.h"
#include "Move.h"

/******Mode********/
bool debug = false; // Mode debug
short mode = 4;     // activation des PID
/******************/

unsigned long timeSetup;

/******ECHANTILLONAGE********/
float dt = 10e-3; // fréquence d'échantillonnage peut être diminuer
volatile int Update_IT = 0;
/****************************/

/******CONSIGNES PID**********/
float cmd_vitesse_G = 0; // commande vitesse moteur gauche en mm/ms
float cmd_vitesse_D = 0; // commande vitesse moteur droite en mm/ms
float cmd_angle = 0;     // commande angle
float cmd_distance = 0;  // commande distance
float VMax = 500;
float distance_final = 0;
float angle_final = 0;
/*****************************/

/***********Etalonnage Encodeur 1m et 10 PI******/
//float distance_encoder_gauche = 1000.0 / 5023.0;
float distance_encoder_gauche = 1000.0 / 5000.0;
float distance_encoder_droit = 1000.0 / 4815.0;

/**************************************/

/********Coef Vitesse ******/
float correction_vitesse = 1.032;
float coefVitesseG = distance_encoder_gauche / dt * correction_vitesse;
float coefVitesseD = distance_encoder_droit / dt;
/**************************/

/********Coef Angle****/
float correction_angle = 0.947;
float empattementRoueCodeuse = 241;
float coefAngle = dt / empattementRoueCodeuse * correction_angle;
/**********************/

/*****ETAT DEPLACEMENT************/
bool distance_ok = false;
bool angle_ok = false;
MovementResult newCommand;
bool send_new_command_available = false;
int arret_lidar = 2; // 0 = arret en cours, 1 = arreter, 2 = reprise possible du mouvement
/*********************************/

/******ENCODEUR************/
int16_t last_encGauche = 0;                           // sauvegarde de la position de l'encodeur gauche
int16_t last_encDroit = 0;                            // sauvegarde de la position de l'encodeur droit

int16_t encGauche_depart = 0;  // [MODIF] ticks init gauche
int16_t encDroit_depart = 0;   // [MODIF] ticks init droit

Encoder encGauche(CodGB, CodGA, TIM3, HALFQUAD, 250); // Pour plus de précision utiliser FULLQUAD (2x plus de ticks)
Encoder encDroit(CodDB, CodDA, TIM2, HALFQUAD, 250);  //
/***************************************/

/*****Sauvegarde des positions*****/
float x = 0;
float y = 0;
/**********************************/

/******Constante mesuré************/
float vitesse_G = 0; // vitesse gauche
float vitesse_D = 0; // vitesse droite
float angle = 0;     // angle
float distance = 0;  // distance
/**********************************/

/******Corection PID************/
float Output_PID_vitesse_G = 0; // Valeur sortante du PID vitesse moteur gauche, une PMW donc
float Output_PID_vitesse_D = 0; // Valeur sortante du PID vitesse moteur droit, une PMW donc
float Output_PID_angle = 0;     // Valeur sortante du PID angle
float Output_PID_distance = 0;  // Valeur sortante du PID distance
/*******************************/

/******COEFICIENTS PID************/
// float Kp_G = 100.0 / 475.0, Ki_G = 0.0, Kd_G = 0.00;        // coefficients PID vitesse moteur gauche
// float Kp_D = 100.0 / 500.0, Ki_D = 0.0, Kd_D = 0.00;        // coefficients PID vitesse moteur droit
// float Kp_angle = 3500, Ki_angle = 1600, Kd_angle = 0;       // coefficients PID angle
// float Kp_distance = 20, Ki_distance = 1.5, Kd_distance = 0; // coefficients PID distance

float Kp_G = 0.200, Ki_G = 0.1, Kd_G = 0.001;        // coefficients PID vitesse moteur gauche
float Kp_D = 0.155, Ki_D = 0.01, Kd_D = 0.001;        // coefficients PID vitesse moteur droit
float Kp_angle = 3500, Ki_angle = 1600, Kd_angle = 0;       // coefficients PID angle
float Kp_distance = 23, Ki_distance = 3, Kd_distance = 0; // coefficients PID distance
/*********************************/

/******Declaration des PID************/
PID PID_vitesse_G(&vitesse_G, &Output_PID_vitesse_G, &cmd_vitesse_G, dt, Kp_G, Ki_G, Kd_G, DIRECT);
PID PID_vitesse_D(&vitesse_D, &Output_PID_vitesse_D, &cmd_vitesse_D, dt, Kp_D, Ki_D, Kd_D, DIRECT);
PID PID_angle(&angle, &Output_PID_angle, &cmd_angle, dt, Kp_angle, Ki_angle, Kd_angle, DIRECT);
PID PID_distance(&distance, &Output_PID_distance, &cmd_distance, dt, Kp_distance, Ki_distance, Kd_distance, DIRECT);
/*************************************/

TypeCommande commande_en_pause = AUCUNE;


/********************************************/
/********************************************/
/********************************************/

/*************************************/
/*****SETUP***************************/
/*************************************/
void setup()
{
  /*****INITIALISATION COMMUNICATION SÉRIE******/
  Serial.begin(115200); // Par défaut utilisation de USART1
  /*********************************************/
  Serial.println("Serial OK");

  /******Initialisation des PINs****/
  pinMode(DIR1, OUTPUT);        // PA_3 = pin D0
  pinMode(DIR2, OUTPUT);        // PA_2 = pin D1
  pinMode(PWM1, OUTPUT);        // PWM4/1 pin D10 donc le Timer4
  pinMode(PWM2, OUTPUT);        // PWM1/1 pin D7 donc le Timer1
  pinMode(LED_BUILTIN, OUTPUT); // Configure la broche de la LED comme sortie
  // pinMode(ARU, INPUT_PULLUP);   // PA_0 = pin D2
  // /*********************************/

  // attachInterrupt(digitalPinToInterrupt(ARU), ARU_interrupt, CHANGE); // Interruption pour l'arrêt d'urgence

  /******Initialisation des encodeurs****/
  if (encDroit.init() && encGauche.init())
  {
    Serial.println("-Encoder Initialization OK");
  }
  else
  {
    Serial.println("-Encoder Initialization Failed");
    while (1)
      ;
  }
  /***************************************/

  digitalWrite(LED_BUILTIN, HIGH); // Allume LED Confirmation d'initialisation

  /******Initialisation de l'interruption pour l'échantillonnage************/
  NVIC_SetPriority(TIM5_IRQn, 1); // Priorité pour l'interruption du timer
  TIM_TypeDef *Instance = TIM5;
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1 / dt, HERTZ_FORMAT);
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
  /**************************************************************************/

  /******Limites des PID************/
  PID_angle.SetOutputLimits(-1000, 1000, 0);
  PID_distance.SetOutputLimits(-1000, 1000, 0);
  PID_vitesse_D.SetOutputLimits(-1000, 1000, 10);
  PID_vitesse_G.SetOutputLimits(-1000, 1000, 10);
  /***************************************/

  /******Reset des Codeurs************/
  encDroit.resetTicks();
  encGauche.resetTicks();
  /***********************************/

  change_PID_mode(4); // Activation des PID

  timeSetup = millis(); // Sauvegarde du temps de démarrage
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****LOOP**************************/
/*************************************/
void loop()
{
  // si on est en mode debug et 100ms se sont écoulées
  if (Update_IT >= 10 && debug)
  {
    sendData(); // Envoi des données
  }

  // si on est pas en phase d'arret du lidar et qu'une nouvelle commande est disponible
  if (send_new_command_available && arret_lidar >= 2)
  {
    encGauche_depart = encGauche.getTicks();
    encDroit_depart = encDroit.getTicks();

    sendData(); // Envoi des données
    // Serial.print("G"); // Consigne de vitesse moteur Gauche
    // Serial.println(cmd_vitesse_G, 5);
    // Serial.print("H"); // Consigne de vitesse moteur Droit
    // Serial.println(cmd_vitesse_D, 5);
    // Serial.print("E"); // Sortie du PID vitesse moteur Gauche
    // Serial.println(Output_PID_vitesse_G, 5);
    // Serial.print("F"); // Sortie du PID vitesse moteur Droit
    // Serial.println(Output_PID_vitesse_D, 5);
    // Serial.print("J"); // angle PID
    // Serial.println(Output_PID_angle);
    // Serial.print("M"); // distance PID
    // Serial.println(Output_PID_distance, 5);
    Serial.println("Z"); // Envoi de l'information qu'on peut recevoir une nouvelle commande
    Serial.flush();
    send_new_command_available = false;
  }
  if (distance_ok && angle_ok)
  {
    int ticksG = encGauche.getTicks();
    int ticksD = encDroit.getTicks();
    Serial.println("TICKS_RESULT:" + String(ticksG - encGauche_depart) + "," + String(ticksD - encDroit_depart));


    // Pour ne pas répéter indéfiniment
    distance_ok = false;
    angle_ok = false;
  }
  if (Serial.available())
    {
        // Serial.println("[DEBUG] Donnée série détectée !");
        // Serial.flush();  // Pour s'assurer que ce message arrive bien
        serialEvent();   // 💡 Appel obligatoire pour lire les commandes (ex: stopmove, restartmove)
    }
}
/*************************************/
/*************************************/
/*************************************/
