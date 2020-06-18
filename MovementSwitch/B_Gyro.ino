


//*******************************************************************************************************************
// Gyro Data Processor       ****************************************************************************************
//*******************************************************************************************************************

bool B_GyroRead(byte cmd)     // returns 0 if ok & 1 if problem
{
float  f1,f2;
static float zero_offset_f      = 0.0;
static float output_filtered_f  = 0.0;

#define NOD_BASE_GAIN   10000.0      
#define PRIMARY_FILER       0.1        // 0.001 -> 1.0 (off)

  mpu.dmpGetQuaternion        (&q, fifoBuffer);
  mpu.dmpGetGravity           (&gravity, &q);
  mpu.dmpGetYawPitchRoll      (RNT, &q, &gravity);

  //   temp_RTN[ROTATE]  = (int)(RNT[ROTATE]* rotate_base_gain);                            // NOT USED
  //   temp_RTN[TILT]    = (int)(RNT[TILT]  * tilt_base_gain);                              // NOT USED
  
  f1 = RNT[NOD] * NOD_BASE_GAIN;                                                            // apply fixed gain to enable transfer to int
  if (cmd == GYRO_ZERO)                                                                     // capture present value to use as an offset & return
   { 
   zero_offset_f = f1;
   if (abs(f1) > 5000.0) { return 1; }                                                      // correction offset rather high so flag as an issue
   return 0; 
   }         
                             
  f1 = f1 - zero_offset_f;                                                                  // apply offset
  output_filtered_f = output_filtered_f + ((f1 - output_filtered_f) * PRIMARY_FILER );      // apply filter
  
  if (cmd != GYRO_NO_ZERO_TRACK)                                                            // slow track zero offset if enabled from main state machine
   {
   if (output_filtered_f > 0) { zero_offset_f += AUTO_CENTRE_RATE ; } else { zero_offset_f -= AUTO_CENTRE_RATE ; }
   }

#ifdef POLARITY_R 
  tilt_output  = -(int)(f1);                                                                 // load output as integer wrt to polarity of operation
#else
  tilt_output  =  (int)(f1); 
#endif
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
