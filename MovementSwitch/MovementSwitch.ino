/*
Version:      1.00  16/6/20

Open Source License - please leave this header intact.

Project:      Tilt Switch Sensor based on the MPU650 Gyro
Originator:   Celtic Magic     www.celticmagic.org
Sponsored by: Ace Centre UK

Credits:
Thanks and credit given to Gyro  Libraries from Jeff Rowberg & the Arduino community.
Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
8/24/2011 by Jeff Rowberg <jeff@rowberg.net>

Legal bit:
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/



//*******************************************************************************************************************
//***************  User Config      *********************************************************************************
//*******************************************************************************************************************

  #define KEY_CHAR                      'A'         // Keyboard charecter when pressed - enter ASCII number for charector eg 97 = a     or enclose for same result ie 'a'      32 = Space
  #define POLARITY_N                                // _N = Normal  press down to click  OR    _R  for the reverse operation
  #define SENSITIVITY                     0         // 1-1000     if set to zero then sensitivity pot used from pin AD0
  #define AUTO_CENTRE_RATE              1.0         // 0.1 - 5.0  this is how strong the auto centering works 

  #define OVER_TRAVEL_OFF_COLOUR        YEL
  #define OFF_COLOUR                    WHT
  #define ON_COLOUR                     GRN
  #define OVER_TRAVEL_ON_COLOUR         GRN
  
    // valid colour codes     BLK,    RED,    GRN,    BLU,    YEL,    PUR,    CYN,  WHT,  WHTmax
    // Tip: if the Over Travel colours are not required then set them the same as the regular On / Off colours
    // Recommend that at least the OVER_TRAVLE_OFF_COLOUR is used as if consistently operating it suggests that the sensitivity can be set lower.

  
  

  
  
  
  








  
  
  
  
//  #define PRE_RELEASE_VERSION

//*******************************************************************************************************************
//***************  Libraries        *********************************************************************************
//*******************************************************************************************************************

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <math.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include "Keyboard.h"
MPU6050 mpu;


//*******************************************************************************************************************
//***************  System Constants *********************************************************************************
//*******************************************************************************************************************

  #define FALSE                    0
  #define TRUE                     1
  #define OFF                      0
  #define ON                       1
  #define NUL                     -1
  #define _10BIT                1024
  #define HALF_10BIT             512
  #define _8BIT                  255
  #define SPAN_8bit              255
  #define HALF_8BIT              127
  
  #define ROTATE                    0
  #define NOD                       1
  #define TILT                      2

  const long MPLEX_INTERVAL_xxMS  = 5;     
  enum {RUN_SM, RESET_SM };
  enum {GYRO_NORMAL, GYRO_ZERO, GYRO_NO_ZERO_TRACK};


//*******************************************************************************************************************
// System Global Variables  *****************************************************************************************
//*******************************************************************************************************************
  static long PreviousMillis        = 0;
  static int  tilt_output           = 0;
  static int  trig_threshold        = 0;


//*******************************************************************************************************************
// MPU6050 Definitions     *****************************************************************************************
//*******************************************************************************************************************
  bool blinkState = false;
  // MPU control/status vars
  bool      dmpReady = false;  // set true if DMP init was successful
  uint8_t   mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t   devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t  packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t  fifoCount;     // count of all bytes currently in FIFO
  uint8_t   fifoBuffer[64]; // FIFO storage buffer
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float RNT[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  // packet structure for InvenSense teapot demo
  uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


//*******************************************************************************************************************
// Hardware Definitions     *****************************************************************************************
//*******************************************************************************************************************

  #define RGB_LED_PIN     9
  #define PB_INPUT_PIN    4
  #define INTERRUPT_PIN   7  
  #define POT_AO_GAIN    A0
  #define NUM_PIXEL_LEDS  1
  #define RELAY_OUT       5
  
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXEL_LEDS, RGB_LED_PIN , NEO_GRB + NEO_KHZ800);


//*************************************************************************************************
// MONITOR LIGHT **********************************************************************************
//*************************************************************************************************

  //Shorthands
  #define  d0              di[0]
  #define  d1              di[1]
  #define  d2              di[2]
  #define  d3              di[3]
  #define  d4              di[4]
  #define  d5              di[5]
  #define  d6              di[6]
  #define  d7              di[7]
  
  #define DEB_ARRAY_SIZE  8
  static int di[DEB_ARRAY_SIZE];          // main debug array (ints)
  static int di_shadow[DEB_ARRAY_SIZE];




//*************************************************************************************************
// APP DATA       *********************************************************************************
//*************************************************************************************************


// LED colours
  #define NO_COLOURS 10
  #define BRIGHTNESS 100
  #define xb  *BRIGHTNESS)/100
  //                                     0       1       2       3       4       5       6       7       8     9
    enum                              { BLK,    RED,    GRN,    BLU,    YEL,    PUR,    CYN,    BLK2,   WHT,  WHTmax  };
    static const byte R[NO_COLOURS] = {(000 xb,(255 xb,(000 xb,(000 xb,(255 xb,(255 xb,(000 xb,(000 xb,(127 xb,(255 xb };
    static const byte G[NO_COLOURS] = {(000 xb,(000 xb,(255 xb,(000 xb,(255 xb,(000 xb,(255 xb,(000 xb,(127 xb,(255 xb };
    static const byte B[NO_COLOURS] = {(000 xb,(000 xb,(000 xb,(255 xb,(000 xb,(255 xb,(255 xb,(000 xb,(127 xb,(255 xb }; 
 


//*************************************************************************************************
// MAIN          **********************************************************************************
//*************************************************************************************************

void setup()
{
  F_MonitorLightInit(115200);
  HW_ConfigInit();
  B_Gyro_setup();
  Keyboard.begin();
}


void loop()
{
  static byte  mplex_10ms = 0;
  unsigned long CurrentMillis;
  
  // FreeRunning Fast Stuff here *****
  B_Gyro_Interrupt_Processor();
  // Main Multiplexer Controller Tick 1mS intervals >>>>>>>>>>>>>>>>>>>>>>>>>>>
  CurrentMillis = millis();
  if (CurrentMillis - PreviousMillis > MPLEX_INTERVAL_xxMS)
   {
   PreviousMillis = CurrentMillis;
   mplex_10ms++;    // 0...4 for ever
   if (mplex_10ms > 4)  { mplex_10ms = 0; }
   SM_10ms(mplex_10ms);
   }
}


// Mutiplexer - called every XXms
//*************************************************************************

void SM_10ms(char mplex)
{
  A_Gryo_SM(RUN_SM);      //    <<<<<<<<<<<<<<<<< The Main Code runs here
  
  switch (mplex)
  {
    case 0:
      {
        F_MonitorLightRun();
      break;
      }
    case 1:
      {
      if (SENSITIVITY > 0)  { trig_threshold  = 1050 - SENSITIVITY; }               // use sensitivity value from header
      else                  { trig_threshold  = 1050 - analogRead(POT_AO_GAIN); }   // or use A0 input
      break;
      }
    case 2:
      {
      if (!digitalRead(PB_INPUT_PIN)) { A_Gryo_SM(RESET_SM); }    
      break;
      }
    case 3:
      {
      // spare  
      break;
      }
    case 4:
      {
      // spare          
      break;
      }
  }
}


//*******************************************************************************************************************
// Interrupt detection       ****************************************************************************************
//*******************************************************************************************************************

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
 {
 mpuInterrupt = true;
 }

//*******************************************************************************************************************
// Configure I/O directions  ****************************************************************************************
//*******************************************************************************************************************

void HW_ConfigInit (void)
{
  pinMode(RGB_LED_PIN,    OUTPUT);
  pinMode(RELAY_OUT,      OUTPUT); 
  pinMode(INTERRUPT_PIN,  INPUT);
  pinMode(2,              INPUT_PULLUP);   // force wire / I2C to use pull ups
  pinMode(3,              INPUT_PULLUP);  
  pinMode(PB_INPUT_PIN,   INPUT_PULLUP);
  pinMode(POT_AO_GAIN ,   INPUT);
}
