#ifndef MAXBOTIX_H
#define MAXBOTIX_H

#include <Arduino.h>
#include <SoftwareSerial.h>

void begin_maxbotix(SoftwareSerial &port, int _EN_pin);
void read_maxbotix(char *buffer, int len_buffer);
int read_maxbotix_int();



#endif // Include guard
