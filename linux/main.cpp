/**
 * @file       main.cpp
 * @authors    John Arena, Shamim Babul, Rona Kosumi, Ionut Rotariu
 * @license    This project is NOT released under any license.
 * @date       Sept 2019-May 2020
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
    
  pinMode(5, INPUT); // GPIO 5 reserved for Speedometer, pin 29
  tmr.setInterval(1000L,readSpeedometerSignal(5)); // Call every second

}


void loop()
{
    Blynk.run();
    tmr.run();
}

/* OUR FUNCTIONS, VARIABLES, ETC BELOW*/

#ifndef OUR_FUNCTION_HEADERS_
#define OUR_FUNCTION_HEADERS_
void readSpeedometerSignal(int pinNumber);
void speedometerFunction();
void speedometerReadingCalculation(double totalTime);
#endif

#include <time.h> /* Will be used for MPH */
#include <cmath>

int wheelSensorGoLowCounter = 1;
double timeDifferenceSeconds = 0.0, milesPerHour = 0.0;
time_t currentTime_1, currentTime_2;


/* DECLARE GLOBAL VARIABLES, LIBRARIES AND PIN MODES ABOVE HERE. WRITE FUNCTIONS BELOW */

void readSpeedometerSignal(int pinNumber){
  if(digitalRead(pinNumber) == LOW){ // Active Low Hall Sensor
	  speedometerFunction();
    }
}

void speedometerFunction(){
  if(wheelSensorGoLowCounter == 1){
    time(&currentTime_1); // sets currentTime_1 to current time
    wheelSensorGoLowCounter++;
  }
  else if(wheelSensorGoLowCounter == 2){
    time(&currentTime_2);
    timeDifferenceSeconds = difftime(currentTime_2,currentTime_1);
    speedometerReadingCalculation(timeDifferenceSeconds);
    Blynk.virtualWrite(V5,milesPerHour);
    time(&currentTime_1);
    wheelSensorGoLowCounter = 1;
  }
}

void speedometerReadingCalculation(double totalTime){
	milesPerHour = (2 * M_PI*(1.083) * 60 * 60) / (5280 * totalTime);
}

int main(int argc, char* argv[])
{
    parse_options(argc, argv, auth, serv, port);

    setup();
    while(true) {
        loop();
    }

    return 0;
}

