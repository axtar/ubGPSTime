// MIT license
// Copyright 2021 highvoltglow

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
// and associated documentation files (the "Software"), to deal in the Software without restriction, 
// including without limitation the rights to use, copy, modify, merge, publish, distribute, 
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
// is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies 
// or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
// OTHER DEALINGS IN THE SOFTWARE.


#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ubGPSTime.h>

// serial pins
#define PIN_GPSTX   32
#define PIN_GPSRX   33

// callback forward definition
void onGPSMessage(UBXMESSAGE *message);

// fix type mapping (see u-blox documentation)
String gpsFixType[] = {"no fix","dead reckoning only", "2D-fix", "3D-fix", "GPS + dead reckoning combined", "Time only fix", "Reserved"};

// globals
SoftwareSerial gpsCom(PIN_GPSRX, PIN_GPSTX);
ubGPSTime gps;

// setup
void setup() 
{
  Serial.begin(115200);
  
  // use default speed
  gpsCom.begin(9600);

  // define com port
  gps.begin(gpsCom);

  // uncomment to get some extra information
  // gps.enableVerbose(Serial);

  // give some time to init the com port
  delay(500);

  // initialize 
  gps.initialize();
  if(gps.isInitialized())
  {
    // display some gps module information
    Serial.println();
    Serial.println("GPS module successfully initialized");
    Serial.print("Software version: ");
    Serial.println(gps.getModuleVersion().swVersion);
    Serial.print("Hardware version: ");
    Serial.println(gps.getModuleVersion().hwVersion);
    for(uint8_t i=0; i<MAX_EXTENSIONS; i++)
    {
      Serial.printf("Extension %u: ", i+1);
      Serial.println(gps.getModuleVersion().extensions[i]);
    }

    // set notification callback 
    // you can stop getting notificatons by calling the detach function
    gps.attach(onGPSMessage);

    // subscribe to messages
    // rate in seconds, use rate = 0 to stop subscription
    gps.subscribeGPSStatus(5); 
    gps.subscribeTimeUTC(1);

    // if you switch off the gps module, all configuation changes 
    // (subscriptions, NMEA unsubscriptions,...) will be lost and 
    // the module will restart with the default configuration 

    // if you only wan't a single response, use the request functions
    // request functions will return without waiting for a response
    // the gps module will discard requests if too many (> 5?) are sent in a row
    // gps.requestStatus();
    // gps.requestTimeUTC();
  }
  else
  {
    Serial.println("Failed to initialize GPS module");
  }
}

// main loop
void loop() 
{
  // process gps messages if initialized
  if(gps.isInitialized())
  {
    gps.process();
  }
}

// message event, time to access current data
void onGPSMessage(UBXMESSAGE *message)
{
  // UBX messages are defined by class and ID
  if(message->msgClass == UBX_NAV)
  {
    switch(message->msgID)
    {
      case UBX_NAV_TIMEUTC:
        Serial.printf("%02u.%02u.%04u %02u:%02u:%02u UTC_valid=%s Date_valid=%s, Time_valid=%s, ms_since_last_update=%lu\n", 
          gps.getTimeUTC().day,
          gps.getTimeUTC().month,
          gps.getTimeUTC().year,
          gps.getTimeUTC().hour,
          gps.getTimeUTC().minute,
          gps.getTimeUTC().seconds,
          gps.getTimeUTC().utcValid ? "yes" : "no",
          gps.getTimeUTC().weekNumberValid ? "yes" : "no",
          gps.getTimeUTC().timeOfWeekValid ? "yes" : "no",
          millis() - gps.getTimeUTC().timestamp); // we can check the age of the last update
        break;

      case UBX_NAV_STATUS:
          String s;
          // map gps fix type
          if(gps.getGPSStatus().gpsFixType < 7)
          {
            s = gpsFixType[gps.getGPSStatus().gpsFixType];
          }
          else
          {
            s = gpsFixType[6];
          }
          Serial.printf("GPSFixOK=%s GPSFixType=%s\n",
            gps.getGPSStatus().gpsFixOk ? "yes" : "no",
            s.c_str());
        break;
    }
  }
}
