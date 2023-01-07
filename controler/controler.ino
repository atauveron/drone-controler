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

#define SIZE 128

// Define where the CSN Pin in connected.
const int slaveSelectPin = 10;

// PWM pin
const int pwmPin = 9;
const int commandPin = 5;

// Radio pins
// Deux pin d'interruption pour pas que les signaux de deux cannaux ne se superposent
const int radioAPin = 2;
// const int radioBPin = 3;

//temps depuis le début du pic
volatile unsigned long temps_chA = 0;
volatile unsigned long temps_chB = 0;

//valeur (entre 1000 et 2000)
int chA_valeur = 0;
int chB_valeur = 0;

//ETAT PRÉCÉDENT DU CANAL (pour gérer 2 canaux sur le même pin d'interruption)
volatile bool etat_chA = 0;
volatile bool etat_chB = 0;

// Motor command
long command = 0;

// Start connection to the angle sensor.
AS5X47 as5047d(slaveSelectPin);

void setup() {
  // Initialize a Serial Communication for printing
  Serial.begin(1000000);

  // Pins D9 and D10 - 7.8 kHz
  // TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00000010;
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 127);
  pinMode(commandPin, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(pwmPin), ISR_ppm, RISING);

  // Interruptions for the radio receiver
  attachInterrupt(digitalPinToInterrupt(radioAPin), ISR_RadioA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(radioBPin), ISR_ppm, CHANGE);
}

void loop() {
  float angles[SIZE];
  unsigned long dates[SIZE];

  for (int i = 0; i < SIZE; ++i) {
    // Read the measured angle
    const float angle = as5047d.readAngle();
    // Show the measured angle
    // Serial.print("Angle: ");
    // Serial.print(micros());
    // Serial.print(",");
    // Serial.println(angle);
    angles[i] = angle;
    dates[i] = micros();

    // Show the inputs from the radio receiver
    // Serial.print("Radio inputs: ");
    // Serial.print(chA_valeur);
    // Serial.print(" ");
    // Serial.println(chB_valeur);

    // Send motor command
    command = min(max(map(chA_valeur, 1080, 2000, 128, 255), 128), 255);
    // Serial.print("Command: ");
    // Serial.println(command);
    analogWrite(pwmPin, command);
  }

  for (int i = 0; i < SIZE; ++i) {
    Serial.print(dates[i]);
    Serial.print(",");
    Serial.println(angles[i]);
  }

  // Wait before reading again.
  // delay(1);
}

void ISR_RadioA() {
  const unsigned long t = micros();

  if (digitalRead(radioAPin)) {
    temps_chA = t;
    etat_chA = 1;
  } else if (etat_chA == 1) {
    chA_valeur = t - temps_chA;
    etat_chA = 0;
  }
}

// void ISR_RadioB() { //ch6 (pin5)  ch4(pin7)  ch2(pin 9)
//   const unsigned long t = micros();
//   if (digitalRead(radioBPin)) {
//     temps_chB = t;
//     etat_chB = 1;
//   }
//   else if (etat_chB == 1) {
//     chB_valeur = t - temps_chB;
//     etat_chB = 0;
//   }
// }

void ISR_ppm() {
  digitalWrite(commandPin, HIGH);
  delayMicroseconds(command);
  digitalWrite(commandPin, LOW);
}
