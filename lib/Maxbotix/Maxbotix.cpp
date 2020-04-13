#include <Maxbotix.h>

SoftwareSerial *mySerial;
int EN_pin;
void begin_maxbotix(SoftwareSerial* port, int _EN_pin)
{
  mySerial = port;
  EN_pin = _EN_pin;
  pinMode(EN_pin, OUTPUT);
  digitalWrite(EN_pin, LOW);
}

void read_maxbotix(char *buffer, int len_buffer)
{
  mySerial->listen();
  int i = 0;
  char incoming_char;
  while(mySerial->available() != 0)
    incoming_char = mySerial->read();
    
  digitalWrite(EN_pin, HIGH);
  delay(200);
  int chars_available = mySerial->available();
  while(chars_available) {
    if((incoming_char = mySerial->read()) != 13) {
      buffer[i] = incoming_char; 
    } else {
      buffer[i] = '\0';
      digitalWrite(EN_pin, LOW);
      break;
    }
    chars_available--;
    i++;
  }
  
  while(mySerial->available() != 0)
    incoming_char = mySerial->read();
}

int read_maxbotix_int()
{
  char data_buffer[10];
  read_maxbotix(data_buffer, 10);
  int i;
  for (i = 1; i < 10; ++i)
    data_buffer[i-1] = data_buffer[i];
  
  
  return atoi(data_buffer);
}
