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

/* OUR FUNCTIONS, VARIABLES, ETC BELOW*/

#ifndef OUR_FUNCTION_HEADERS_
#define OUR_FUNCTION_HEADERS_
void readSpeedometerSignal();
void speedometerFunction();
void speedometerReadingCalculation(double totalTime);
#endif

#include <time.h> /* Will be used for MPH */
#include <cmath>
#include <iostream>
#include <iomanip>      
//using namespace std;

int wheelSensorGoLowCounter = 1;
double timeDifferenceSeconds = 0.0, milesPerHour = 0.0;
double totalTime;
clock_t currentTime_1, currentTime_2;
int gpioSpeedometer = 12;

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
    
  pinMode(12, INPUT); // GPIO 12, pin 32
  tmr.setInterval(90L,readSpeedometerSignal); // Call every .09 seconds
  //tmr.setInterval(5000L, readPIN); // Testing how to read pin function

}

void loop()
{
    Blynk.run();
    tmr.run();
}

/* DECLARE GLOBAL VARIABLES, LIBRARIES AND PIN MODES ABOVE HERE. WRITE FUNCTIONS BELOW */

void readSpeedometerSignal(){
  if(digitalRead(gpioSpeedometer) == 0){ // Active Low Hall Sensor
	  speedometerFunction();
	  std::cout << "GPIO PIN is LOW" << std::endl;
    }
}

void speedometerFunction(){
  if(wheelSensorGoLowCounter == 1){
	currentTime_1 = clock(); // Records time of reading
	std::cout << "time 1: " << currentTime_1 << std::endl;
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	wheelSensorGoLowCounter++;
  }
  else if (wheelSensorGoLowCounter > 1 && wheelSensorGoLowCounter < 10) {
	  wheelSensorGoLowCounter++;
  }
  else if(wheelSensorGoLowCounter == 10){
	currentTime_2 = clock();
	std::cout << "time 2: " << currentTime_2 << std::endl;
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	totalTime = (currentTime_2 - currentTime_1) * 1e-3; //milliseconds to seconds
	std::cout << "timeDifferenceSeconds:" << totalTime  << std::endl;
    speedometerReadingCalculation(totalTime);
	std::cout << "MPH:" << milesPerHour << std::endl;
    Blynk.virtualWrite(V12,milesPerHour);
	wheelSensorGoLowCounter = 1;
  }
  else{}
}

void speedometerReadingCalculation(double totalTime){
	milesPerHour = (5* 2 * M_PI*(1.083) * 60 * 60) / (5280 * totalTime);
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

