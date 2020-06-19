

//*******************************************************************************************************************
// Main State Machine       *****************************************************************************************
//*******************************************************************************************************************

byte A_Gryo_SM (byte cmd)
{
enum {SM_BOOT_0, SM_INIT, SM_RESET_0,SM_RESET_1, SM_OFF, SM_OFF_ON_0, SM_ON_0, SM_ON_OFF_0};  
static int  a_gyro_timer  = 0;
static byte a_gyro_sm     = SM_BOOT_0;

d7 = a_gyro_sm;

  if (cmd == RESET_SM)  { a_gyro_sm = SM_RESET_0; }
  if (a_gyro_timer > 0) { a_gyro_timer--; }

 switch (a_gyro_sm)
  {
  case SM_BOOT_0:   
   { 
   A_Led_Driver(100, RED); 
   GyroProcessorInit();    
   a_gyro_timer = 100;
   a_gyro_sm = SM_INIT;
   digitalWrite(RELAY_OUT, HIGH);                                // drive external relay output OFF (high)    
   break;
   }

  case SM_INIT: 
   { 
   if (a_gyro_timer > 0)                                         // wait for Gyro to settle on boot
    {
    B_GyroRead(GYRO_NORMAL);  
    break; 
    } 
   a_gyro_sm = SM_RESET_0;  
   break;
   }

  case SM_RESET_0:
   {
   if (B_GyroRead(GYRO_ZERO)) {a_gyro_sm = SM_BOOT_0; break; }   // problem encountered try roboot
   A_Led_Driver(100, PUR);   
   a_gyro_timer = 30; 
   a_gyro_sm = SM_RESET_1;  
   break;  
   }

  case SM_RESET_1:                                                // wait to display Reset colour 
   {
   B_GyroRead(GYRO_ZERO);
   if (a_gyro_timer > 0)  { break; }
   a_gyro_sm = SM_OFF; 
   break;
   }
   
  case SM_OFF: 
   {    
   B_GyroRead(GYRO_NORMAL);
   if (tilt_output < -trig_threshold) { A_Led_Driver(100, OVER_TRAVEL_OFF_COLOUR ); }   // trigger in opposite direction - do nowt but indicate yellow
   else                               { A_Led_Driver(100, OFF_COLOUR); }   // good to go waiting trigger
   if ( tilt_output > trig_threshold) { a_gyro_sm = SM_OFF_ON_0; }  // trigger ON    
   break;
   }
   
  case SM_OFF_ON_0:   
   {
   B_GyroRead(GYRO_ZERO);                                                   // centre on new ON position
#ifndef PRE_RELEASE_VERSION
   Keyboard.write(KEY_CHAR);                                                // send single key press 
#endif   
   digitalWrite(RELAY_OUT, LOW);                                            // drive external relay output ON (low)
   a_gyro_sm = SM_ON_0;
   break;
   }
   
  case SM_ON_0:   
   {
   if (tilt_output >  trig_threshold) { A_Led_Driver(100, OVER_TRAVEL_ON_COLOUR  ); }
   else                               { A_Led_Driver(100, ON_COLOUR ); }
   B_GyroRead(GYRO_NORMAL);                                                 // continue to auto zero when ON
   if ( tilt_output < -trig_threshold) { a_gyro_sm = SM_ON_OFF_0; }         // look for OFF condition
   break;
   }
   
  case SM_ON_OFF_0:  
   {
   digitalWrite(RELAY_OUT, HIGH);                                           // drive external relay output OFF (high)    
   B_GyroRead(GYRO_ZERO);                                                   // zero to OFF
   a_gyro_sm = SM_OFF;
   break;
   }
  }
return 0;
}




//*******************************************************************************************************************
// Led driver               *****************************************************************************************
//*******************************************************************************************************************

void A_Led_Driver(byte intensity, byte colour)
{
uint32_t c;   
 c = strip.Color((intensity*R[colour])/SPAN_8bit,(intensity*G[colour])/SPAN_8bit,(intensity*B[colour])/SPAN_8bit);  // calc 24bit colour & intensity   
 strip.setPixelColor(0, c);     
 strip.show();
}



         
