// #include "interruption.h"
// #include "Odometrie.h"
// #include <Arduino.h>
// #include "main.h"

// float epsilonDistance = 5;         // erreur acceptable pour la distance
// float epsilonAngle = PI / 180 * 2.5; // erreur acceptable pour l'angle
// unsigned int interrupt_tick = 0;   // temps écoulé depuis le début de la commande

// /************************************************************************************/
// /*************************Explication des fonction***********************************/
// /************************************************************************************
//  * Update_IT_callback: Fonction d'interruption pour l'échantiollnage du PID.
//  * Elle est appelé toute les dt secondes par le timer. Cette fonction permet de
//  * calculer les vitesses des moteurs, l'angle et la distance parcourue par le robot,
//  * ainsi que de calculer les sorties des PID. Pour l'asservissement en postion du robot.
//  *************************************************************************************
//  * ARU_interrupt: Fonction d'interruption pour l'arrêt d'urgence. Elle est appelé
//  * lorque le bouton d'arret d'urgence est appuyé. Cette fonction permet de redémarer
//  * la carte de zéro. Un problème éléctronique fait que l'interuption n'était pas appelé
//  * lors de l'appui sur le bouton d'arrêt d'urgence.
//  ************************************************************************************/
// /************************************************************************************/
// /************************************************************************************/

// /*************************************/
// /*****FONCTION ÉCHANTILLONAGE*********/
// /*************************************/
// void Update_IT_callback(void)
// {
//   /****Récupération des valeurs des codeurs****/
//   int16_t ticks_G = (encGauche.getTicks()); // évite tous changment de valeur des ticks pendant l'interuption
//   int16_t ticks_D = (encDroit.getTicks());
//   /********************************************/

//   if (distance_ok && angle_ok && newCommand.goto_ok || newCommand.rotate_ok && angle_ok)
//   {
//     change_PID_mode(4); // on met les PID en mode auto au cas ou ils étaient en manuel

//     /***Réinitialisation des valeurs*****/

//     // Reset de la disatnce
//     reset_last_distance();
//     cmd_distance = 0;
//     distance = 0;

//     // Supression de l'erreur entre la commande et l'angle
//     cmd_angle = angle;

//     // Reset du temps
//     interrupt_tick = 0;
//     /************************************/

//     /*******Verification pour éviter les débordement des timers*****/
//     // si les ticks sont trop grand, on les remet à 0
//     // cette solution a été implémenté rapidement et nous nous sommes rendu compte
//     // que cela entrainait une erreur au niveau du calcul de l'odometrie,
//     // avec l'apparition d'un leger décalage de la position.
//     // Solution a améliorer !
//     if (encDroit.getTicks() > 2500 || encGauche.getTicks() > 2500)
//     {
//       last_encGauche = -ticks_D + last_encDroit;
//       last_encDroit = -ticks_D + last_encDroit;
//       encDroit.resetTicks();
//       encGauche.resetTicks();
//       ticks_G = 0;
//       ticks_D = 0;
//     }

//     angle_ok = false;
//     Serial.println("![TRACE] angle_ok = false (nouvelle commande)");
//     if (newCommand.goto_ok)
//       distance_ok = false;
//       Serial.println("![TRACE] distance_ok = false (nouvelle commande goto)");

//     newCommand.goto_ok = false;
//   }
//   // Si l'angle n'est pas atteint, on continue de tourner
//   else if (!angle_ok)
//   {

//     interrupt_tick += 1;                            // on incrémente le temps
//     cmd_angle = angle_command_ramp(interrupt_tick); // on récupère la commande de l'angle
//   }
//   // une fois l'angle atteint, on passe à la distance
//   else if (!distance_ok && angle_ok)
//   {
//     interrupt_tick += 1;                                  // on incrémente le temps
//     cmd_distance = distance_command_ramp(interrupt_tick); // on récupère la commande de la distance
//   }
//   /******************************************************************/

//   /****Calcul des vitesses des moteurs*******/
//   vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
//   vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;
//   /******************************************/

//   /****Calcul de l'angle et de la distance*******/
//   angle += (vitesse_G - vitesse_D) * coefAngle;
//   distance += (vitesse_D + vitesse_G) / 2 * dt;
//   /*********************************************/

//   /*****Calul de PID Angle et Vitesse****/
//   // On commence par ajuster l'angle, puis la distance
//   // si il a atteint la consigne en distance ou que le temps qu'il doit mettre pour atteindre la consigne est dépassé
//   //if ((abs(distance_final - distance) < epsilonDistance || interrupt_tick >= get_distance_tf()) && angle_ok && !distance_ok)

//   if (((abs(distance_final - distance) < epsilonDistance && abs((vitesse_G + vitesse_D) / 2.0) < 10.0)
//      || interrupt_tick >= get_distance_tf())
//     && angle_ok && !distance_ok)

//   {
//     // Serial.println("[DEBUG] Vérification fin de distance");
//     // Serial.print("distance_final = ");
//     // Serial.println(distance_final);
//     // Serial.print("distance = ");
//     // Serial.println(distance);
//     // Serial.print("distance erreur = ");
//     // Serial.println(abs(distance_final - distance));

//     // Serial.print("vitesse moyenne = ");
//     // Serial.println((vitesse_G + vitesse_D) / 2.0);

//     // Serial.print("interrupt_tick = ");
//     // Serial.println(interrupt_tick);
//     // Serial.print("get_distance_tf() = ");
//     // Serial.println(get_distance_tf());

//     // Serial.print("angle_ok = ");
//     // Serial.println(angle_ok);
//     // Serial.print("distance_ok = ");
//     // Serial.println(distance_ok);


//     distance_ok = true;            // accepte une nouvelle commande de distance
//     Serial.println("![TRACE] distance_ok = true (distance atteinte)");

//     arret_lidar++;                 // on incrémente le nombre de fois ou on a pas reçu de commande
//     // Serial.print("[DEBUG] arret_lidar incrementé à : ");
//     // Serial.println(arret_lidar);
//     newCommand.distance_final = 0; // on remet la commande de distance à 0, au cas ou on reçoit un restartmove
//     newCommand.distance_initial = 0;
//     send_new_command_available = true; // on envoie un message pour dire qu'on est prêt à recevoir une nouvelle commande
//     reset_time_distance();             // on remet les temps des rampes à 0
//     interrupt_tick = 0;                // on remet le temps actuelle à 0
    
//     // 🔥 PATCH : figer les consignes une fois terminé
//     // Output_PID_distance = 0;
//     // cmd_distance = distance;

//     //Serial.println("Z");
//     Serial.println("[DEBUG] CONDITION DE FIN DE DISTANCE REMPLIE");

//   }
//   else
//     PID_distance.Compute(); // on calcule la sortie du PID distance

//   if ((abs(angle_final - angle) < epsilonAngle || interrupt_tick >= get_angle_tf()) && !angle_ok)
//   {
//     if (newCommand.rotate_ok)
//     {
//       newCommand.rotate_ok = false;
//       send_new_command_available = true;
//     }
//     angle_ok = true;
  

//     //PATCH AJOUT
//     // Output_PID_angle = 0;
//     // cmd_angle = angle;

//     reset_time_angle();
//     interrupt_tick = 0;

//     Serial.println("![DEBUG] CONDITION DE FIN DE ROTATE REMPLIE");

//   }
//   else
//     PID_angle.Compute(); // on calcule la sortie du PID angle
//   /*************************************/

//   if (distance_ok && angle_ok && send_new_command_available)
// {
//     Serial.println("Z");
//     Serial.println("![DEBUG] CONDITION DE FIN ACTION ENTIÈREMENT REMPLIE");
//     send_new_command_available = false;
// }
//   // si l'erreur dans la distance ou l'angle est trop grande, on ne fait rien
//   // évite l'emballement des roue une fois arriver à sa condition.
//   if (((abs(cmd_angle - angle) > 3 * PI / 180) && angle_ok && distance_ok || (abs(cmd_distance - distance) > 10) && distance_ok && angle_ok) && PID_angle.GetMode())
//   {
//     change_PID_mode(0);
//     Output_PID_angle = 0;
//     Output_PID_distance = 0;
//     Output_PID_vitesse_D = 0;
//     Output_PID_vitesse_G = 0;
//   }

  

//   /***Ajustement Commandes Vitesse****/
//   cmd_vitesse_G = +Output_PID_distance + Output_PID_angle;
//   cmd_vitesse_D = +Output_PID_distance - Output_PID_angle;
//   /***********************************/

//   /****Calcul des PID Vitesse*******/
//   PID_vitesse_G.Compute();
//   PID_vitesse_D.Compute();
//   /*********************************/

//   /****Commande de direction des moteurs*******/
//   // si la vitesse est positive HIGH, sinon LOW
//   digitalWriteFast(DIR1, (Output_PID_vitesse_D >= 0));
//   digitalWriteFast(DIR2, (Output_PID_vitesse_G >= 0));
//   /********************************************/

//   /****Commande des moteurs ajout avec une deadzone*******/
//   if (abs(Output_PID_vitesse_G) > 10)
//     analogWrite(PWM1, abs(Output_PID_vitesse_G));
//   else
//     analogWrite(PWM1, 0);

//   if (abs(Output_PID_vitesse_D) > 10)
//     analogWrite(PWM2, abs(Output_PID_vitesse_D));
//   else
//     analogWrite(PWM2, 0);
//   /********************************************************/

//   /****Calcul de la position*******/
//   update_Position(distance, angle);
//   /*******************************/

//   /****Sauvegarde des positions*****/
//   last_encGauche = ticks_G;
//   last_encDroit = ticks_D;
//   /********************************/

//   /****FIGER LES PID VITESSE quand l'action est terminée*****/
//   // if (distance_ok && angle_ok)
//   // {
//   //   Output_PID_vitesse_D = 0;
//   //   Output_PID_vitesse_G = 0;
//   // }

//   Update_IT++; // incrémente le nombre d'interruption pour l'envoie du debug
// }
// /*************************************/
// /*************************************/
// /*************************************/

// /*************************************/
// /***********INTERUPTION***************/
// /*********ARRET D'URGENCE*************/
// void ARU_interrupt()
// {
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(500);
//   HAL_NVIC_SystemReset(); // redèmare le programme
// }
// /*************************************/
// /*************************************/
// /*************************************/

#include "interruption.h"
#include "Odometrie.h"
#include <Arduino.h>
#include "main.h"
#include "move.h" // pour accéder à stop_now

float epsilonDistance = 5;
float epsilonAngle = PI / 180 * 2.5;
unsigned int interrupt_tick = 0;

void Update_IT_callback(void)
{
  /**** STOP GLOBAL SI stop_now == true ****/
  if (stop_now)
  {
    Output_PID_angle = 0;
    Output_PID_distance = 0;
    Output_PID_vitesse_D = 0;
    Output_PID_vitesse_G = 0;

    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);

    Serial.println("![DEBUG] stop_now actif → interruption ignorée, moteurs arrêtés.");
    return;
  }

  /**** Récupération des ticks codeurs ****/
  int16_t ticks_G = (encGauche.getTicks());
  int16_t ticks_D = (encDroit.getTicks());

  if ((distance_ok && angle_ok && newCommand.goto_ok) || (newCommand.rotate_ok && angle_ok))
  {
    change_PID_mode(4);

    reset_last_distance();
    cmd_distance = 0;
    distance = 0;

    cmd_angle = angle;
    interrupt_tick = 0;

    if (encDroit.getTicks() > 2500 || encGauche.getTicks() > 2500)
    {
      last_encGauche = -ticks_D + last_encDroit;
      last_encDroit = -ticks_D + last_encDroit;
      encDroit.resetTicks();
      encGauche.resetTicks();
      ticks_G = 0;
      ticks_D = 0;
    }

    angle_ok = false;
    Serial.println("![TRACE] angle_ok = false (nouvelle commande)");

    if (newCommand.goto_ok)
    {
      distance_ok = false;
      Serial.println("![TRACE] distance_ok = false (nouvelle commande goto)");
    }

    newCommand.goto_ok = false;
  }
  else if (!angle_ok)
  {
    interrupt_tick += 1;
    cmd_angle = angle_command_ramp(interrupt_tick);
  }
  else if (!distance_ok && angle_ok)
  {
    interrupt_tick += 1;
    cmd_distance = distance_command_ramp(interrupt_tick);
  }

  /**** Calcul des vitesses et odométrie ****/
  vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
  vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;

  angle += (vitesse_G - vitesse_D) * coefAngle;
  distance += (vitesse_D + vitesse_G) / 2 * dt;

  if (((abs(distance_final - distance) < epsilonDistance && abs((vitesse_G + vitesse_D) / 2.0) < 10.0)
       || interrupt_tick >= get_distance_tf())
      && angle_ok && !distance_ok)
  {
    distance_ok = true;
    Serial.println("![TRACE] distance_ok = true (distance atteinte)");
    arret_lidar++;

    newCommand.distance_final = 0;
    newCommand.distance_initial = 0;

    send_new_command_available = true;
    reset_time_distance();
    interrupt_tick = 0;

    Serial.println("[DEBUG] CONDITION DE FIN DE DISTANCE REMPLIE");
  }
  else
  {
    PID_distance.Compute();
  }

  if ((abs(angle_final - angle) < epsilonAngle || interrupt_tick >= get_angle_tf()) && !angle_ok)
  {
    if (newCommand.rotate_ok)
    {
      newCommand.rotate_ok = false;
      send_new_command_available = true;
    }

    angle_ok = true;
    reset_time_angle();
    interrupt_tick = 0;
    Serial.println("![DEBUG] CONDITION DE FIN DE ROTATE REMPLIE");
  }
  else
  {
    PID_angle.Compute();
  }

  if (distance_ok && angle_ok && send_new_command_available)
  {
    Serial.println("Z");
    Serial.println("![DEBUG] CONDITION DE FIN ACTION ENTIÈREMENT REMPLIE");
    send_new_command_available = false;
  }

  if (((abs(cmd_angle - angle) > 3 * PI / 180) && angle_ok && distance_ok)
      || (abs(cmd_distance - distance) > 10 && distance_ok && angle_ok))
  {
    if (PID_angle.GetMode())
    {
      change_PID_mode(0);
      Output_PID_angle = 0;
      Output_PID_distance = 0;
      Output_PID_vitesse_D = 0;
      Output_PID_vitesse_G = 0;
    }
  }

  cmd_vitesse_G = +Output_PID_distance + Output_PID_angle;
  cmd_vitesse_D = +Output_PID_distance - Output_PID_angle;

  PID_vitesse_G.Compute();
  PID_vitesse_D.Compute();

  digitalWriteFast(DIR1, (Output_PID_vitesse_D >= 0));
  digitalWriteFast(DIR2, (Output_PID_vitesse_G >= 0));

  if (abs(Output_PID_vitesse_G) > 10)
    analogWrite(PWM1, abs(Output_PID_vitesse_G));
  else
    analogWrite(PWM1, 0);

  if (abs(Output_PID_vitesse_D) > 10)
    analogWrite(PWM2, abs(Output_PID_vitesse_D));
  else
    analogWrite(PWM2, 0);

  update_Position(distance, angle);

  last_encGauche = ticks_G;
  last_encDroit = ticks_D;

  Update_IT++;
}

/*************************************/
void ARU_interrupt()
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  HAL_NVIC_SystemReset();
}
