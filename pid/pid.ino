#include "Arduino.h"
#include "pidClass.h"
#define ENCODER_LEFT 2
#define ENCODER_RIGHT 7
#define ENCODER_CON 10

// String to hold incoming serial data
String data_string = "";  

// Motor objects for LeftMotor, RightMotor, and converyer belt
pidClass leftMotor = pidClass(3, 4, 2, 12*74.83);    // Pins for LeftMotor: in1 = 3, in2 = 4, encoder_pin = 2, encoder_ticks_per_rev = 1200
pidClass RightMotor = pidClass(5, 6, 7, 12*74.83);   // Pins for RightMotor: in1 = 5, in2 = 6, encoder_pin = 7, encoder_ticks_per_rev = 1200
pidClass conMotor = pidClass(8, 9, 10, 12*74.83);   // Pins for MiddleMotor: in1 = 8, in2 = 9, encoder_pin = 10, encoder_ticks_per_rev = 1200

// setup interrupts
void left_encoder_isr(){
  leftMotor.encoder_ISR();
}

void right_encoder_isr(){
  rightMotor.encoder_ISR();
}

void con_encoder_isr(){
  conMotor.encoder_ISR();
}
void setup() {
  // USB communication
  Serial.begin(115200);
  Serial.println("Serial Comm Via USB UART");
  Serial.flush();

  // establish encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), left_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), right_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CON), con_encoder_isr, RISING);

}

void loop() {
  // Read data from serial and build the input string
  while (Serial.available()) {
    char byte = (char)Serial.read();
    data_string += byte; // could have "\n" at the end of the string

    // Check if the string ends with newline indicating a complete command
    if (byte == '\n') {
      parseAndExecuteCommand(data_string);  // Handle the incoming command
      data_string = "";  // Clear the string for the next command
    }
  }
}

// Function to parse and execute commands for motors
void parseAndExecuteCommand(String command) {
  char motor = command.charAt(0);  // Get motor identifier (L, R, M)
  int speed = command.substring(1).toInt();  // Get the speed (after the motor identifier)

  // Control the correct motor based on the input
  if (motor == 'L') {
    LeftMotor.control(speed);  // Control LeftMotor
    Serial.print("LeftMotor set to: ");
  } 
  else if (motor == 'R') {
    RightMotor.control(speed);  // Control RightMotor
    Serial.print("RightMotor set to: ");
  } 
  else if (motor == 'M') {
    MiddleMotor.control(speed);  // Control MiddleMotor
    Serial.print("MiddleMotor set to: ");
  }
  else {
    Serial.println("Invalid motor identifier.");
    return;
  }

  // Print the received command and speed
  Serial.println(speed);
}
