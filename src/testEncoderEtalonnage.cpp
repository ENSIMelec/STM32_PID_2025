// #include "Arduino.h"
// #include "FastInterruptEncoder.h"

// // Pins moteurs/codeurs (adapte selon ton montage)
// #define PWM1 PA8 // PWM4/1 pin D10 donc le Timer4
// #define DIR1 PA10
// #define PWM2 PB6 // PWM1/1 pin D7 donc le Timer1
// #define DIR2 PB3
// #define CodDB PA0 // codeur droit chanel B
// #define CodDA PA1 // codeur droit chanel A
// #define CodGB PB4
// #define CodGA PB5
// #define ARU PA14 // Arret d'urgence

// Encoder encGauche(CodGB, CodGA, TIM3, HALFQUAD, 250);
// Encoder encDroit(CodDB, CodDA, TIM2, HALFQUAD, 250);

// float distance_encoder_gauche = 1000.0 / 5000.0; // mm / ticks
// float distance_encoder_droit  = 1000.0 / 4815.0; // mm / ticks

// bool mouvementTermine = false;

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Test encodeur pour 1 mètre");

//   pinMode(DIR1, OUTPUT);
//   pinMode(DIR2, OUTPUT);
//   pinMode(PWM1, OUTPUT);
//   pinMode(PWM2, OUTPUT);

//   if (encDroit.init() && encGauche.init()) {
//     Serial.println("Encodeurs OK");
//   } else {
//     Serial.println("Erreur encodeur");
//     while (1);
//   }

//   encDroit.resetTicks();
//   encGauche.resetTicks();

//   delay(1000); // Pause avant départ

//   digitalWrite(DIR1, 1); // Avant (à adapter selon ton robot)
//   digitalWrite(DIR2, 1);
//   delay(10);

//   analogWrite(PWM1, 100); // Valeur PWM ajustable
//   analogWrite(PWM2, 100);

//   Serial.println("Départ...");
// }

// void loop() {
//   if (!mouvementTermine) {
//     int16_t ticksG = encGauche.getTicks();
//     int16_t ticksD = encDroit.getTicks();

//     float distanceG = ticksG * distance_encoder_gauche;
//     float distanceD = ticksD * distance_encoder_droit;

//     float distanceMoy = (distanceG + distanceD) / 2.0;

//     if (distanceMoy >= 1000.0) {
//       analogWrite(PWM1, 0);
//       analogWrite(PWM2, 0);
//       mouvementTermine = true;

//       Serial.println("1 mètre atteint !");
//       Serial.print("Ticks Gauche : ");
//       Serial.println(ticksG);
//       Serial.print("Ticks Droit  : ");
//       Serial.println(ticksD);
//     }
//   }
// }
