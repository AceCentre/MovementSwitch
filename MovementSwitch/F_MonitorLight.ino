// ************************************************************************************************
// MONITOR LIGHT **********************************************************************************
// ************************************************************************************************


#define MONITOR_FAST_UPDATE_OFF

void F_MonitorLightRun(void)
{
#ifdef PRE_RELEASE_VERSION 

static byte index = 0;
static byte MonWriteIndexer = 0;
static bool freeze = FALSE;


// Command parse .................................................

if (Serial.available())
  {AAAAAAA

   if (Serial.peek() == 'f'  )   
    {  
    Serial.read();  
    if (freeze == FALSE)  { freeze = TRUE;  Serial.println("FREEZE"); } 
    else                  { freeze = FALSE; Serial.println("RUNNING"); } 
    }

   // Parse commands 
   if (Serial.peek() == 'd'  )       // load index for writable debug var
    {
      Serial.read();                            //flush command from buffer
  //    delay(3);                                 //wait for serial data to fully come in
      MonWriteIndexer = F_Mon_helper_get_value(); // now read & load data
      if ( MonWriteIndexer < 0)    {  MonWriteIndexer = 0;  }
      if ( MonWriteIndexer > 7)    {  MonWriteIndexer = 7;  }
      goto mon_update;  // refresh now
    }

    
   //    delay(6);            //wait for serial data to fully come in
       di[MonWriteIndexer] = F_Mon_helper_get_value();  
  }
   
 
  if (freeze == TRUE)           { return; }     
     Serial.read();    //clear any rubbish


  
#ifdef MONITOR_FAST_UPDATE_ON  
// Auto refresh if variable has changed
      for (int i=0; i < DEB_ARRAY_SIZE; i++)  
        {  
        if (di[i] != di_shadow[i])
         {
         di_shadow[i] = di[i];
         goto mon_update;
         }
        }
return;  
#else
  if (index > 0) { --index; } else { index = (DEB_ARRAY_SIZE-1); }
  if (di[index] == di_shadow[index])  { return; }
  di_shadow[index] = di[index];
#endif
    
mon_update:
      for (int i=0; i < DEB_ARRAY_SIZE; i++)  
        {  
        if (MonWriteIndexer == i)          {  Serial.print(">");  }
        else                               {  Serial.print(" ");  }
        F_Mon_helper_print_int(di[i]); 
        }
      Serial.println();      

#endif      
}
  
//*****************************
int F_Mon_helper_get_value()
{
  char buffer[7];
  if (Serial.available() > 0) 
  {    
    for (int i=0; i < 6; i++)     { buffer[i] = Serial.read();    }    // load buffers with serial data
    if (Serial.read() != -1)      { goto DONE_helper_get_value;   }                   // if data is here then overflow occured so NUL data   
    Serial.flush();
    return (atoi(buffer));
  }
DONE_helper_get_value:
return 0;
}

//*****************************
void F_Mon_helper_print_int(int val)
{
     Serial.print(val, DEC);     
//Normalise spacing
     if (val >= 0)    {  Serial.print(F("     ")); }    //allow for neg sign
     else             {  Serial.print(F("    "));  }
     val = abs(val);     
     if (val >= 10000)      {                          return;  }
     if (val >= 1000)       { Serial.print(F(" "));    return;  }
     if (val >= 100)        { Serial.print(F("  "));   return;  }   
     if (val >= 10)         { Serial.print(F("   "));  return;  }   
                              Serial.print(F("    ")); return;  
}


//*************************************************************************************************************
// INIT
//*************************************************************************************************************

void F_MonitorLightInit(long baud)
{
  Serial.begin(baud);
}
