/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined TB6612FNG
  void initMotorController() {
  // Pines de dirección como salida
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);

  // Canal 0 para motor izquierdo
  ledcSetup(0, 5000, 8); // canal 0, 1kHz, 8 bits
  ledcAttachPin(LEFT_MOTOR_ENABLE, 0); 

  // Canal 1 para motor derecho
  ledcSetup(1, 5000, 8); 
  ledcAttachPin(RIGHT_MOTOR_ENABLE, 1);  
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255){
      spd = 255;
    }

    if (spd == 0){
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
      digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
      digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
    }

    else {
      if (i == LEFT) { 
        if      (reverse == 0) { 
          ledcWrite(0, spd); 
          digitalWrite(LEFT_MOTOR_FORWARD, HIGH); 
          digitalWrite(LEFT_MOTOR_BACKWARD, LOW); 
          }
        else if (reverse == 1) { 
          ledcWrite(0, spd); 
          digitalWrite(LEFT_MOTOR_FORWARD, LOW); 
          digitalWrite(LEFT_MOTOR_BACKWARD, HIGH); 
          }
      }
      else /*if (i == RIGHT) //no need for condition*/ {
        if      (reverse == 0) { 
          ledcWrite(1, spd); 
          digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); 
          digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); 
        }
        else if (reverse == 1) { 
          ledcWrite(1, spd); 
          digitalWrite(RIGHT_MOTOR_FORWARD, LOW); 
          digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH); 
        }
      }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
