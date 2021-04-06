   

//*******************************************************************************************************************
// Main State Machine       *****************************************************************************************
//*******************************************************************************************************************

    
byte A_Gryo_SM (byte cmd)
{
 //   0          1          2           3           4               5               6       7             8         9
enum {SM_BOOT_0, SM_INIT,   SM_RESET_0, SM_RESET_1, SM_3D_DETECT_1, SM_3D_DETECT_2, SM_OFF, SM_OFF_ON_0,  SM_ON_0,  SM_ON_OFF_0};  
static int  a_gyro_timer  = 0;
static byte a_gyro_sm     = SM_BOOT_0;

/* 
 // dignostics
d7 = a_gyro_sm;
d1 = tilt_output;
d2 = polarity_3d;
d6 = trig_threshold;
*/

  if (cmd == RESET_SM)      { a_gyro_sm = SM_RESET_0; }
  if (cmd == FULL_RESET_SM) { a_gyro_sm = SM_BOOT_0; }
  if (a_gyro_timer > 0)     { a_gyro_timer--; }

 switch (a_gyro_sm)
  {
  case SM_BOOT_0:   
   { 
   if (detection_style == DETECT_STYLE_3D)    { A_Led_Driver(100, PUR); }
   else                                       { A_Led_Driver(100, RED); }
   if (cmd == FULL_RESET_SM)                  { A_Led_Driver(0,   BLK); }
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
   if (detection_style == DETECT_STYLE_3D)    { A_Led_Driver(100, PUR); }
   else                                       { A_Led_Driver(100, RED); }
   a_gyro_timer = 30; 
   polarity_3d  = 1;
   a_gyro_sm = SM_RESET_1;  
   break;  
   }

  case SM_RESET_1:                                                // wait to display Reset colour 
   {
   B_GyroRead(GYRO_ZERO);
   if (a_gyro_timer > 0)  { break; }
   if (detection_style == DETECT_STYLE_3D)  {   a_gyro_sm = SM_3D_DETECT_1;  }
   else                                     {   a_gyro_sm = SM_OFF;  }
   break;
   }

  case SM_3D_DETECT_1:
   {
   #define FLASH_RATE 50
   if (a_gyro_timer == 0) { a_gyro_timer = FLASH_RATE; }                        // flash indicatio wiating for 3D catch
   if (a_gyro_timer > (FLASH_RATE/2))   { A_Led_Driver(100, OFF_COLOUR); } 
   else                                 { A_Led_Driver(50,  OFF_COLOUR); } 
   B_GyroRead(GYRO_CAPTURE_POLARITIES);   
   if ((abs(tilt_output)) > trig_threshold)
    {

    if (tilt_output < 0) { polarity_3d = -1;  }  // correct polarity wrt to 3D vector
    B_GyroRead(GYRO_ZERO);                       // centre on new ON position
    a_gyro_timer = 20;                           // delay to allow data to stalise in next case
    B_GyroRead(GYRO_LOAD_POLARITIES);  
    a_gyro_sm = SM_3D_DETECT_2;                        
    }
   break;
   }

  case SM_3D_DETECT_2:
   {
   A_Led_Driver(100, ON_COLOUR); 
   if (a_gyro_timer > 0) { break; }
   B_GyroRead(GYRO_ZERO);   
   a_gyro_sm = SM_ON_0; 
   break;
   }
   
  case SM_OFF: 
   {    
   B_GyroRead(GYRO_NORMAL);
   if ( polarity_3d == 1)
    {
    if (tilt_output < -trig_threshold) { A_Led_Driver(100, OVER_TRAVEL_OFF_COLOUR ); }   // trigger in opposite direction - do nowt but indicate yellow
    else                               { A_Led_Driver(100, OFF_COLOUR); }                // good to go waiting trigger
    if ( tilt_output > trig_threshold) { a_gyro_sm = SM_OFF_ON_0; }  // trigger ON    
    }
   else
    {
    if (tilt_output >   trig_threshold) { A_Led_Driver(100, OVER_TRAVEL_OFF_COLOUR ); }   // trigger in opposite direction - do nowt but indicate yellow
    else                                { A_Led_Driver(100, OFF_COLOUR); }                // good to go waiting trigger
    if ( tilt_output < -trig_threshold) { a_gyro_sm = SM_OFF_ON_0; }  // trigger ON      
    }
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
  if ( polarity_3d == 1)
   {
   if ( tilt_output < -trig_threshold) { a_gyro_sm = SM_ON_OFF_0; }         // look for OFF condition
   }
  else
   {
   if ( tilt_output > trig_threshold) { a_gyro_sm = SM_ON_OFF_0; }         // look for OFF condition
   }
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



//*******************************************************************************************************************
// USER Mode Detection Changer   ************************************************************************************
//*******************************************************************************************************************
// Return TRUE if mode change is running

byte B_UserModeChanger()
{
#define EE_PTR 10                      
#define BIT_PAT_2D    0x5A
#define BIT_PAT_3D    0xA5
#define TOGGLE_RATE   16000
static byte sm = 0;
static int  timer = 0;
int t0;

if (timer > 0) {timer--; }

switch (sm)
 {
  case 0:
   {
   if (!digitalRead(PB_INPUT_PIN)) {  timer = 3000; sm++; break;} 
   t0 = EEPROM.read(EE_PTR);
   if ( t0 == BIT_PAT_2D )  {   detection_style = DETECT_STYLE_2D;  return FALSE; }       // x5A & xA5 are 10101 bit patterns used as flags
   if ( t0 == BIT_PAT_3D )  {   detection_style = DETECT_STYLE_3D;  return FALSE; }  
   // force EE mode if not either of the above for next time
   EEPROM.write(EE_PTR,BIT_PAT_2D);
   return FALSE;   
   }

  case 1:
   {
   if (timer > 0) { break; }
   if (!digitalRead(PB_INPUT_PIN)) {  timer = TOGGLE_RATE; sm++; break;}  // still hold PB after a time
   return FALSE;      
   break;
   }

  case 2:
  {
  A_Led_Driver(40, RED);   // 2D
  if (digitalRead(PB_INPUT_PIN)) {    EEPROM.write(EE_PTR,BIT_PAT_2D );  detection_style = DETECT_STYLE_2D; return FALSE; }   // 2D selected on PB release     
  if (timer > 0) { break; }
  timer = TOGGLE_RATE;
  sm++;  
  break;  
  }

  case 3:
  {
  A_Led_Driver(40, PUR);   // 3D
  if (digitalRead(PB_INPUT_PIN)) {    EEPROM.write(EE_PTR,BIT_PAT_3D );  detection_style = DETECT_STYLE_3D; return FALSE; }   // 3D selected on PB release     
  if (timer > 0) { break; }
  timer = TOGGLE_RATE;
  sm--;  
  break;  
  }
 }

return TRUE; 
}




         
