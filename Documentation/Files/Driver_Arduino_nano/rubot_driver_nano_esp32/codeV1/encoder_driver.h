/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A D12  //pin 2
  #define LEFT_ENC_PIN_B D10  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A D11  //pin A4
  #define RIGHT_ENC_PIN_B D9   //pin A5
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

