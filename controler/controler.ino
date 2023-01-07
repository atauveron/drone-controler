/*
  ReadAngle
  Read the angle position from an AS5047 or AS5147 rotary encoder.
  Start the terminal with a speed of 9600 Bauds to see the measured angle.
  The encoder is connected as follows:
  AS5X47         Arduino Board
    5V <-------------> 5V
   GND <-------------> GND
  MOSI <-------------> MOSI (Pin 11 for Arduino Uno)
  MISO <-------------> MISO (Pin 12 for Arduino Uno)
   SCK <-------------> SCK  (Pin 13 for Arduino Uno)
   CSN <-------------> SS   Arbitrary, Pin 10 in this example.
*/

// Include the library
#include <AS5X47.h>

// Define where the CSN Pin in connected. 
int slaveSelectPin = 10;

// Deux pin d'interruption pour pas que les signaux de deux cannaux ne se superposent
#define PINRADIOA 2
#define PINRADIOB 3

//temps depuis le début du pic
volatile unsigned long temps_chA = 0;
volatile unsigned long temps_chB = 0;

//valeur (entre 1000 et 2000)
int chA_valeur = 0;
int chB_valeur = 0;

//ETAT PRÉCéDENT DU CANAL (pour gérer 2 canaux sur même pin interruption)
volatile bool etat_chA = 0;
volatile bool etat_chB = 0;

// Start connection to the angle sensor.
AS5X47 as5047d(slaveSelectPin);

void setup() {
  // Initialize a Serial Communication in order to print the measured angle.
  Serial.begin(1000000);

  // Interruptions for the radio receiver
  attachInterrupt(digitalPinToInterrupt(PINRADIOA), ISR_RadioA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINRADIOB), ISR_RadioB, CHANGE);
}

void loop() {
  // Read the measured angle
  unsigned long before = micros();
  float angle = as5047d.readAngle();
  unsigned long after = micros();

  Serial.print("Elapsed (for angle): ");  
  Serial.println(after - before);

  // Show the measured angle
  Serial.print("Angle: ");
  Serial.println(angle);

  // Show the inputs from the radio receiver
  Serial.print("Radio inputs: ");
  Serial.print(chA_valeur);
  Serial.print(" ");
  Serial.println(chB_valeur);

  // Wait before reading again.
  delay(100);
}

void ISR_RadioA() { 
  unsigned long t = micros();

  if (digitalRead(PINRADIOA)) {
    temps_chA = t;
    etat_chA = 1;
  }
  else if (etat_chA == 1) {
    chA_valeur = t - temps_chA;
    etat_chA = 0;
  }
}

void ISR_RadioB() { //ch6 (pin5)  ch4(pin7)  ch2(pin 9)
  unsigned long t = micros();
  if (digitalRead(PINRADIOB)) {
    temps_chB = t;
    etat_chB = 1;
  }
  else if (etat_chB == 1) {
    chB_valeur = t - temps_chB;
    etat_chB = 0;
  }
}
