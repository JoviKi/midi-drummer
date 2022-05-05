#include <string.h>
#include <stdlib.h>
#include "SerialTransfer.h"

SerialTransfer myTransfer;

//Buffer
int buf1 = 3000;
int buf2 = 11;

//Flag
volatile int flag1 = 0;
volatile int flag2 = 0;

double data = 0.0;
uint16_t sendSize = 0;

//Callbackfunctions, die durch mytransfer.tick() aufgerufen werden, wenn neue
//Daten angkommen. Die Funktionen speichern die neuen Werte in buf1 und buf2.
void wrist()
{
  myTransfer.rxObj(buf1);
}

void thumb()
{
  myTransfer.rxObj(buf2);
}
  
// Liste der Callbackfunktionen. Wenn ein Package mit der ID 0 ankommt wird wrist() aufgerufen. 
//Wenn ein Package mit der ID 1 ankommt wird thumb aufgerufen. 
const functionPtr callbackArr[] = { wrist, thumb };


void setup()
{

  // Starten und Konfiguration der Seriellen Schnittstelle.
  Serial.begin(115200);
  configST myConfig;
  myConfig.debug        = true;
  myConfig.callbacks    = callbackArr;
  myConfig.callbacksLen = sizeof(callbackArr) / sizeof(functionPtr);  
  myTransfer.begin(Serial, myConfig);

  
  //Konfiguration Timer 1 für Handgelenk
  TCCR1A = (1<<COM1B1) + (1<<WGM10) + (1<<WGM11); // Set OC2A at bottom, clear OC2B at compare match
  TCCR1B = (1<<CS11) + (1<<WGM13); 
  TIMSK1 = (1<<OCIE1A);
  OCR1A = 20000;
  OCR1B = 1500;
  DDRB |= (1<<PB2);

  //Konfiguration Timer 2 für Daumen
  TCCR2A = (1<<COM2B1) + (1<<WGM20); // Set OC2A at bottom, clear OC2B at compare match
  TCCR2B = (1<<CS22) + (1<<CS21) + (1<<CS20) + (1<<WGM22); // prescaler = 1024;
  TIMSK2 = (1<<OCIE2A);
  OCR2A = 120;//156; //50Hz Update
  OCR2B = 11; //7-16
  DDRD |= (1<<PD3);
  }

void loop()
{

  if(flag1 == 1) {
      //data=1.0 als ID fürs Handgelenk. 
      data = 1.0;
      //Sende Nachricht über die Serielle Schnittstelle und warte 1 Millisikunde
      sendSize = myTransfer.txObj(data, sendSize);
      myTransfer.sendData(sendSize);
      delay(1);
      flag1 = 0;
      sendSize = 0;
  }
    
  if(flag2 == 2) {
      data = 2.0;
      sendSize = myTransfer.txObj(data, sendSize);
      myTransfer.sendData(sendSize);
      delay(1);
      flag2 = 0;
      sendSize = 0;

  }
  myTransfer.tick();
  delay(1);

}



ISR (TIMER1_COMPA_vect){  // Interrupt Service Routine
  //Lade neuen OCR1B-Wert auf dem Buffer. Dann setze Flagge, damit ein neuer Wert angefragt wird.
  OCR1B = buf1;
  flag1 = 1;
}

ISR (TIMER2_COMPA_vect){  // Interrupt Service Routine
  OCR2B = buf2;
  flag2 = 2;
}
