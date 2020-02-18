/**
 * @file       main.cpp
 * @author     Volodymyr Shymanskyy
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2015 Volodymyr Shymanskyy
 * @date       Mar 2015
 * @brief
 */

//#define BLYNK_DEBUG
#define BLYNK_PRINT stdout
#ifdef RASPBERRY
  #include <BlynkApiWiringPi.h>
#else
  #include <BlynkApiLinux.h>
#endif
#include <BlynkSocket.h>
#include <BlynkOptionsParser.h>

static BlynkTransportSocket _blynkTransport;
BlynkSocket Blynk(_blynkTransport);

static const char *auth, *serv;
static uint16_t port;

#include <BlynkWidgets.h>

BlynkTimer tmr;

BLYNK_WRITE(V1)
{
    printf("Got a value: %s\n", param[0].asStr());
}

void setup()
{
    Blynk.begin(auth, serv, port);
    tmr.setInterval(1000, [](){
      Blynk.virtualWrite(V0, BlynkMillis()/1000);
    });
}

void loop()
{
    Blynk.run();
    tmr.run();
}

#include <time.h> /* Will be used for MPH */
#include <cmath>
int main(int argc, char* argv[])
{
    parse_options(argc, argv, auth, serv, port);

    setup();
    while(true) {
        loop();
    }

    int wheelSensorGoLowCounter = 1;
    time_t currentTime_1, currentTime_2;
    double timeDifferenceSeconds, milesPerHour;
    if(PIN_8 == LOGIC_LOW){
      if(wheelSensorGoLowCounter == 1){
        time(&currentTime_1); // sets currentTime_1
        wheelSensorGoLowCounter++;
      }
      else if(wheelSensorGoLowCounter == 2){
        time(&currentTime_2)
        timeDifferenceSeconds = difftime(currentTime_2,currentTime_1);
        time(&currentTime_1);
        signalCounter = 1;
      }
      else;
    }

    double speedometerReadingCalculation(double totalTime){
      milesPerHour = (2*M_PI*(1.083)*60*60)/(5280*totalTime)
    }

    return 0;
}

