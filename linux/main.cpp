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
void readPIN();
#endif

#include <time.h> /* Will be used for MPH */
#include <cmath>
#include <iostream>
#include <iomanip>      
using namespace std;

int wheelSensorGoLowCounter = 1;
double timeDifferenceSeconds = 0.0, milesPerHour = 0.0;
//time_t currentTime_1, currentTime_2;
long totalTime;
struct timespec start, end;
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
  tmr.setInterval(500L,readSpeedometerSignal); // Call every .5 seconds
  //tmr.setInterval(5000L, readPIN); // Testing how to read pin function

}

void loop()
{
    Blynk.run();
    tmr.run();
}

/* DECLARE GLOBAL VARIABLES, LIBRARIES AND PIN MODES ABOVE HERE. WRITE FUNCTIONS BELOW */

void readPIN() {
	//if (digitalRead(12) == LOW) {
	//	cout << "GPIO PIN IS LOW" << endl;
	//}
	//else if (digitalRead(12) == HIGH) {
	//	cout << "GPIO PIN IS HIGH" << endl;
	//}
	//else {};
	int pinstatus = digitalRead(12);
	cout << "Pin status of 12 is: " << pinstatus << "." << endl;
}

void readSpeedometerSignal(){
  if(digitalRead(gpioSpeedometer) == 0){ // Active Low Hall Sensor
	  speedometerFunction();
	  cout << "GPIO PIN is LOW" << endl;
    }
}

void speedometerFunction(){
  if(wheelSensorGoLowCounter == 1){
    //time(&currentTime_1); // sets currentTime_1 to current time
	//printf("currentTime_1 %s", ctime(&currentTime_1));
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
	wheelSensorGoLowCounter++;
	cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << endl;
  }
  else if(wheelSensorGoLowCounter == 2){
    //time(&currentTime_2);
	//printf("currentTime_2 %s", ctime(&currentTime_2));
    //timeDifferenceSeconds = difftime(currentTime_2,currentTime_1);
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end);
	//timeDifferenceSeconds = double(currentTime_2 - currentTime_1);
	totalTime = end.tv_nsec - start.tv_nsec;
	cout << "timeDifferenceSeconds:" << totalTime << endl;
    speedometerReadingCalculation(timeDifferenceSeconds);
	cout << "MPH:" << milesPerHour << setprecision(5) << endl;
    Blynk.virtualWrite(V12,milesPerHour);
    //time(&currentTime_1);
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
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

