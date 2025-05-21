#include "Communication.h"

/*****Variabe pour la communication*****/
char inputBuffer[1024] = "\0"; // Buffer pour stocker les données entrantes
int sizeBuffer = 0;            // Taille du buffer
/***************************************/

/*************************************************************************************/
/*************************Explication des fonction************************************/
/*************************************************************************************
 * printUsage: Fonction d'affichage de l'usage des commandes shell. Attention, cette
 * fonction est appelé si la commande n'est pas reconnu, mais elle n'est pas à jour.
 * Il est préférable de consulter le fichier Communication.cpp pour avoir les commandes
 * à jour.
 *************************************************************************************
 * asservCommandUSB: Fonction d'interprétation des commandes shell. Elle permet de
 * transformer les commandes shell en commande pour l'asservissement du robot.
 * Elle est appelé par la fonction usbSerialCallback et prend en argument le nombre
 * d'argument et les arguments.
 *************************************************************************************
 * usbSerialCallback: Fonction de réception des données de l'USB. Elle permet de
 * transformer les données reçu en commande shell pour l'asservissement du robot.
 * Elle sépare les données reçu en plusieurs arguments et les envoie à la fonction
 * asservCommandUSB. Elle est appelé par la fonction serialEvent.
 *************************************************************************************
 * serialEvent: Fonction d'interruption pour la réception des données de l'USB. Elle
 * est appelé à chaque fois qu'une donnée est reçu sur le port USB.
 *************************************************************************************
 * sendData: Fonction d'envoie des données de l'asservissement. Elle permet d'envoyer
 * les données de l'asservissement sur le port USB.
 *************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

/**************************************/
/******EXEMPLE DE COMMANDE SHELL*******/
/**************************************/
// asserv debug [enable|disable]
// asserv recalage [1|0]
// asserv [enable|disable|reset] [all|angle|distance]
// asserv [enable|disable|reset] vitesse [all|gauche|droite]
// asserv set coord [x] [y]
// asserv set angle [angle]
// asserv set position [x] [y] [angle]
// asserv goto [x] [y] [speed]
// asserv rotate [angle]
// asserv moveof [distance] [speed]
// asserv stopmove
// asserv restartmove
/**************************************/

/*************************************/
/******FONCTION AFFICHAGE USAGE*******/
/*************************************/
void printUsage()
{
  char printUsage[2048] = "Usage : \r\n - asserv wheelcalib \r\n - asserv enablemotor [0|1] \r\n - asserv enablepolar [0|1] \r\n - asserv coders \r\n - asserv reset \r\n - asserv motorspeed [r|l] [speed] \r\n -------------- \r\n - asserv wheelspeedstep [r|l] [speed] [step time] \r\n -------------- \r\n - asserv robotfwspeedstep [speed] [step time] \r\n - asserv robotangspeedstep [speed] [step time] \r\n - asserv speedcontrol [r|l] [Kp] [Ki] \r\n - asserv angleacc delta_speed \r\n- asserv distacc delta_speed \r\n ------------------- \r\n - asserv addangle angle_rad \r\n - asserv anglereset \r\n - asserv anglecontrol Kp \r\n------------------- \r\n - asserv adddist mm \r\n - asserv distreset \r\n - asserv distcontrol Kp \r\n -------------- \r\n - asserv addgoto X Y \r\n - asserv gototest \r\n -------------- \r\n - asserv pll freq \r\n";
  Serial.print(printUsage);
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/********FONCTION INTERPRÉTEUSE*******/
/*************************************/
void asservCommandUSB(int argc, char **argv)
{
  // On vérifie le nombre d'argument
  if (argc == 0)
  {
    printUsage();
    return;
  }
  // Activation ou désactivation du debug
  else if (!strcmp(argv[0], "debug"))
  {
    if (!strcmp(argv[1], "enable"))
    {
      debug = true;
    }
    else if (!strcmp(argv[1], "disable"))
    {
      debug = false;
    }
  }
  else if (!strcmp(argv[0], "manette"))
  {
    if (!strcmp(argv[1], "vitesse"))
    {
      Output_PID_distance = min(max(atoi(argv[2]), -500), 500);
      Serial.println("ok");
    }
    else if (!strcmp(argv[1], "angle"))
    {
      Output_PID_angle = min(max(atoi(argv[2]), -350), 350);
      Serial.println("ok");
    }
  }
  else if (!strcmp(argv[0], "recalage"))
  {
    recalage(atoi(argv[1]));
  }
  else if (!strcmp(argv[0], "enable"))
  {
    if (argv[1] == "all")
    {
      change_PID_mode(4); // enable de tout l'asservissement
    }
    else if (argv[1] == "angle")
    {
      change_PID_mode(1); // enable de l'asservissement d'angle
    }
    else if (argv[1] == "distance")
    {
      // enable de l'asservissement de distance
    }
    else if (argv[1] == "vitesse")
    {
      if (argv[2] == "all")
      {
        change_PID_mode(3); // enable de l'asservissement de vitesse droite et gauche
        Serial.println("send");
      }
      else if (argv[2] == "gauche")
      {
        // enable de l'asservissement de vitesse gauche
      }
      else if (argv[2] == "droite")
      {
        // enable de l'asservissement de vitesse droite
      }
    }
  }
  else if (!strcmp(argv[0], "disable"))
  {
    if (argv[1] == "all")
    {
      // disable de tout l'asservissement
    }
    else if (argv[1] == "angle")
    {
      // disable de l'asservissement d'angle
    }
    else if (argv[1] == "distance")
    {
      // disable de l'asservissement de distance
    }
    else if (argv[1] == "vitesse")
    {
      if (argv[2] == "all")
      {
        // disable de l'asservissement de vitesse droite et gauche
      }
      else if (argv[2] == "gauche")
      {
        // disable de l'asservissement de vitesse gauche
      }
      else if (argv[2] == "droite")
      {
        // disable de l'asservissement de vitesse droite
      }
    }
  }
  else if (!strcmp(argv[0], "reset"))
  {
    if (!strcmp(argv[1], "all"))
    {
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      HAL_NVIC_SystemReset(); // redèmare le programme
    }
    else if (argv[1] == "angle")
    {
      angle = 0; // reset de l'asservissement d'angle
    }
    else if (argv[1] == "distance")
    {
      reset_distance(); // reset de l'asservissement de distance
    }
    else if (argv[1] == "vitesse")
    {
      if (argv[2] == "all")
      {
        // reset de l'asservissement de vitesse droite et gauche
      }
      else if (argv[2] == "gauche")
      {
        // reset de l'asservissement de vitesse gauche
      }
      else if (argv[2] == "droite")
      {
        // reset de l'asservissement de vitesse droite
      }
    }
  }
  else if (!strcmp(argv[0], "set"))
  {
    if (!strcmp(argv[1], "coord"))
    {
      x = (float)atof(argv[2]);
      y = (float)atof(argv[3]);
    }
    else if (!strcmp(argv[1], "angle"))
    {
      angle = (float)atof(argv[2]); // set de l'angle
    }
    else if (!strcmp(argv[1], "position"))
    {
      x = (float)atof(argv[2]);
      y = (float)atof(argv[3]);
      angle = (float)atof(argv[4]); // set de l'angle
    }
  }
  else if (!strcmp(argv[0], "goto"))
  {

    if (argc < 3 || !(distance_ok && angle_ok) || arret_lidar < 2)
    {
      Serial.println("Erreur");
      return;
    }
    float x = atof(argv[1]);
    float y = atof(argv[2]);
    newCommand = calculateMovement(x, y);
    if (argc > 4)
    {
      newCommand.speed = atof(argv[3]);
    }
    // goTo(newCommand);
    // newCommand.goto_ok = true;
    newCommand.goto_ok = true;
    commande_en_pause = GOTO;
    goTo(newCommand);

  }
  else if (!strcmp(argv[0], "rotate") && (distance_ok && angle_ok))
  {
    float angle_ = atof(argv[1]);
    newCommand = calculate_rotation(angle_);
    newCommand.speed = 50;
    // rotate(newCommand);
    // newCommand.rotate_ok = true;
    newCommand.rotate_ok = true;
    commande_en_pause = ROTATE;
    rotate(newCommand);

  }
  else if (!strcmp(argv[0], "moveof"))
  {
    // Serial.println(argc);
    if (argc < 2 || !(distance_ok && angle_ok) || arret_lidar < 2)
    {
      Serial.println("Erreur");
      // if (argc<2)Serial.println("Erreur argc");
      // if (distance_ok)Serial.println("Erreur distance_ok");
      // if (angle_ok)Serial.println("Erreur angle_ok");
      // if (arret_lidar <2 )Serial.println("Erreur arret_lidar");
      return;
    }
    float distance_ = atof(argv[1]);
    newCommand = calculate_moveOf(distance_);
    if (argc > 2)
    {
      newCommand.speed = atof(argv[2]);
    }
    // moveOf(newCommand);
    newCommand.goto_ok = true;
    newCommand.moveof_ok = true;
    commande_en_pause = MOVEOF;
    moveOf(newCommand);

  }
  else if (!strcmp(argv[0], "stopmove"))
  {
    arret_lidar = 0;
    obstacle_detection();
  }
  else if (!strcmp(argv[0], "restartmove"))
  {
    // Serial.print("[DEBUG] restartmove reçu avec arret_lidar = ");
    // Serial.println(arret_lidar);
    if (arret_lidar >= 2)
    {
        // Serial.println("[DEBUG] Condition remplie");
        after_obstacle_detection();
        arret_lidar = 0;  // pour réinitialiser l’état
    }
    else
    {
        // Serial.println("[DEBUG] Trop tôt pour reprendre (arret_lidar < 2)");
    }

  }
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/**FONCTION CONVERSION EN ARGC ARGV***/
/*************************************/
void usbSerialCallback(char *buffer, uint32_t size)
{
  // Serial.print("[DEBUG] usbSerialCallback : ");
  // Serial.println(buffer);

  if (size > 0)
  {
    /*
     *  On transforme la commande recu dans une version argv/argc
     *    de manière a utiliser les commandes shell déjà définie...
     */
    bool prevWasSpace = false;
    char *firstArg = buffer;
    int nb_arg = 0;
    char *argv[10];
    for (uint32_t i = 0; i < size; i++)
    {
      if (prevWasSpace && buffer[i] != ' ')
      {
        argv[nb_arg++] = &buffer[i];
      }
      if (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')
      {
        prevWasSpace = true;
        buffer[i] = '\0';
      }
      else
      {
        prevWasSpace = false;
      }
    }

    // On évite de faire appel au shell si le nombre d'arg est mauvais ou si la
    // 1ière commande est mauvaise...
    if (nb_arg > 0 && !strcmp(firstArg, "asserv"))
    {
      asservCommandUSB(nb_arg, argv);
    }
    else
    {
      printUsage();
    }
  }
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****FONCTION RÉCÉPTION DONNÉES******/
/*************************************/
void serialEvent()
{
  //Serial.println("[DEBUG] serialEvent déclenché");
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == '\0')
  {
    usbSerialCallback(inputBuffer, sizeBuffer);
    memset(inputBuffer, '\0', sizeBuffer); // On vide le buffer
    sizeBuffer = 0;
  }
  else
  {
    inputBuffer[sizeBuffer] = c;
    sizeBuffer++;
  }
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*******Envoie des données************/
/*************************************/
void sendData()
{
  // Serial.print("A"); // Valeur du codeur Gauche
  // Serial.println(last_encGauche);
  // Serial.print("B"); // Valeur du codeur Droit
  // Serial.println(last_encDroit);

  // Serial.print("C"); // Vitesse réel moteur Gauche
  // Serial.println(vitesse_G, 5);
  // Serial.print("D"); // Vitesse réel moteur Droit
  // Serial.println(vitesse_D, 5);
  // Serial.print("E"); // Sortie du PID vitesse moteur Gauche
  // Serial.println(Output_PID_vitesse_G, 5);
  // Serial.print("F"); // Sortie du PID vitesse moteur Droit
  // Serial.println(Output_PID_vitesse_D, 5);
  // Serial.print("G"); // Consigne de vitesse moteur Gauche
  // Serial.println(cmd_vitesse_G, 5);
  // Serial.print("H"); // Consigne de vitesse moteur Droit
  // Serial.println(cmd_vitesse_D, 5);
  // Serial.print("I"); // angle mesurer
  // Serial.println(angle, 5);

  // Serial.print("J"); // angle PID
  // Serial.println(Output_PID_angle);

  // Serial.print("K");
  // Serial.println(cmd_angle, 5);
  Serial.print("L"); // distance mesurer
  Serial.println(distance, 5);

  //  Serial.print("M"); // distance PID
  //  Serial.println(Output_PID_distance, 5);

  // Serial.print("O"); // cmd distance
  // Serial.println(cmd_distance, 5);

  // Serial.print("P"); // angle ok
  // Serial.println(angle_ok);
  // Serial.print("Q"); // distance ok
  // Serial.println(distance_ok);
  // Serial.print("X"); // position x
  // Serial.println(x);
  // Serial.print("Y"); // position y
  // Serial.println(y);

  Update_IT = 0;
}
/*************************************/
/*************************************/
/*************************************/