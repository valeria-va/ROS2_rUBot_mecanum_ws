/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 8
  #define LEFT_MOTOR_BACKWARD  7
  #define RIGHT_MOTOR_FORWARD  18
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 47
  #define LEFT_MOTOR_ENABLE 38
#endif

#ifdef TB6612FNG
  #define RIGHT_MOTOR_BACKWARD D6
  #define LEFT_MOTOR_BACKWARD  D4
  #define RIGHT_MOTOR_FORWARD  D3
  #define LEFT_MOTOR_FORWARD   D5
  #define RIGHT_MOTOR_ENABLE D7
  #define LEFT_MOTOR_ENABLE D2
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
