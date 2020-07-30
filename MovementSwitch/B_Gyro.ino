
// Tutorial on using the Gyro..,.
// https://www.mschoeffler.de/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/



bool B_GyroRead(byte cmd)  
{
  if (detection_style == DETECT_STYLE_3D) { return GyroRead_3D(cmd); }      // split call to appriate function
  else                                    { return GyroRead_2D(cmd); }
}


//*******************************************************************************************************************
// Gyro 3D Processor     ****************************************************************************************
//*******************************************************************************************************************


bool GyroRead_3D(byte cmd)     // returns 0 if ok & 1 if problem
{
#define BASE_GAIN_3D     10000.0        // vector arithmetic results in longer diagonal vectors than 2D so compensate with reduced gain
#define PRIMARY_FILTER       0.05        // 0.001 -> 1.0 (off)
enum { RN,RT,NT };  
float  f1;
static float zero_offset_f[3]       = {0.0,0.0,0.0};
static float output_filtered_f[3]   = {0.0,0.0,0.0};
static float output_peak_f[3]       = {0.0,0.0,0.0};
static float base_gain_f[3]         = {BASE_GAIN_3D,BASE_GAIN_3D,BASE_GAIN_3D};
   
  mpu.dmpGetQuaternion        (&q, fifoBuffer);
  mpu.dmpGetGravity           (&gravity, &q);
  mpu.dmpGetYawPitchRoll      (RNT, &q, &gravity);

// Process data & auto centre
   for (int i=0; i < 3; i++)
    {
    f1 = base_gain_f[i] * RNT[i];                                                                                                // apply base gain with polarity 

    if (cmd == GYRO_ZERO)    
     {
     zero_offset_f[i]     = f1;                                                                                                  // capture present offsets
     output_peak_f[i]     = 0.0;                                                                                                 // reset peak capture on GYRO_ZERO
     output_filtered_f[i] = 0.0;
     if (abs(f1) > 5000.0) { return 1; }                                                                                         // correction offset rather high so flag as an issue
     }
    else  // normal run
     {
     f1 = f1 - zero_offset_f[i];                                                                                                  // apply offsets
     output_filtered_f[i] = output_filtered_f[i] + ((f1 - output_filtered_f[i]) * PRIMARY_FILTER );                               // apply filter
     if (output_filtered_f[i] > 0) { zero_offset_f[i] += AUTO_CENTRE_RATE ; } else { zero_offset_f[i] -= AUTO_CENTRE_RATE ; }     // Intgrate to auto zero
     }
    }
    
   if (cmd == GYRO_CAPTURE_POLARITIES)
    {
    for (int i=0; i < 3; i++)
     {      
     if (abs(output_filtered_f[i]) >= abs(output_peak_f[i])) { output_peak_f[i] = output_filtered_f[i]; }     //  store peaks and preserve polarity
     }
    }
    
   if (cmd == GYRO_LOAD_POLARITIES)
    {
    for (int i=0; i < 3; i++)
     {     
     #define REJECT_LEVEL 0.0             
     if (abs(output_peak_f[i]) < REJECT_LEVEL) { base_gain_f[i] = 0.0; }
     else
      {
      if (output_peak_f[i] < 0.0) { base_gain_f[i] = -BASE_GAIN_3D; }                               // polarity corection is part of the gain
      else                        { base_gain_f[i] =  BASE_GAIN_3D; }
      } 
     }
    }
    
  f1 = output_filtered_f[ROTATE] + output_filtered_f[NOD] + output_filtered_f[TILT];               // sum all polarity corected vectors
  tilt_output = int(f1);
return 0;
}



//*******************************************************************************************************************
// Gyro 2D Processor     ****************************************************************************************
//*******************************************************************************************************************



bool GyroRead_2D(byte cmd)     // returns 0 if ok & 1 if problem
{
float  f1,f2;
static float zero_offset_f      = 0.0;
static float output_filtered_f  = 0.0;

#define BASE_GAIN_2D     10000.0      
#define PRIMARY_FILTER       0.1        // 0.001 -> 1.0 (off)

  mpu.dmpGetQuaternion        (&q, fifoBuffer);
  mpu.dmpGetGravity           (&gravity, &q);
  mpu.dmpGetYawPitchRoll      (RNT, &q, &gravity);

  f1 =  BASE_GAIN_2D * RNT[NOD];
  if (cmd == GYRO_ZERO)                                                                     // capture present value to use as an offset & return
   { 
   zero_offset_f = f1;
   if (abs(f1) > 5000.0) { return 1; }                                                      // correction offset rather high so flag as an issue
   return 0; 
   }         
  f1 = f1 - zero_offset_f;                                                                  // apply offset  
  output_filtered_f = output_filtered_f + ((f1 - output_filtered_f) * PRIMARY_FILTER );     // apply filter
  
 
  if (output_filtered_f > 0) { zero_offset_f += AUTO_CENTRE_RATE ; } else { zero_offset_f -= AUTO_CENTRE_RATE ; }
 

  switch (detection_style)
   {
   case DETECT_STYLE_2D:  
    { tilt_output  =  (int)(f1);  break; }
   case DETECT_STYLE_2DR:  
    { tilt_output  = -(int)(f1);  break; }
   case DETECT_STYLE_3D:      // SHOULD NOT HAPPEN
    {   break; }
   }
return 0;
}



//*******************************************************************************************************************
// Gyro ISR                  ****************************************************************************************
//*******************************************************************************************************************


void B_Gyro_Interrupt_Processor ()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)  return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();

        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if(fifoCount < packetSize){
      //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
      gyro_error = TRUE;
#ifdef PRE_RELEASE_VERSION    
        Serial.println(F("FIFO overflow!"));
#endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }
 } 
}


//*******************************************************************************************************************
// Gyro / Wire BAAAAAoot          ****************************************************************************************
//*******************************************************************************************************************


void B_Gyro_setup() 
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
}


//*******************************************************************************************************************
// Gyro Process Boot         ****************************************************************************************
//*******************************************************************************************************************

void GyroProcessorInit ()
{
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    delay(100);  
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
   if (devStatus == 0) 
   {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);     
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();  
   }
}
