#include "main.h"
#include "Odometrie.h"
#include "Move.h"
#include "FastInterruptEncoder.h"


extern Encoder encGauche;
extern Encoder encDroit;

void printUsage();
void asservCommandUSB(int argc, char **argv);
void usbSerialCallback(char *buffer, uint32_t size);
void serialEvent();
void sendData();
