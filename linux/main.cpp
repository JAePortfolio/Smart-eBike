/**
 * g++ llv3.cpp lidarlite_v3.cpp -I . -o llv3.out
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
void UpdateLidar();
#endif

#include <time.h> /* Will be used for MPH */
#include <cmath>
#include <iostream>
#include <iomanip>      
#include <linux/types.h>
#include <cstdio>
#include <lidarlite_v3.h>
#include <iostream>
#include <fstream>
#include <chrono>	
//#include <linux/gpio.h>
#include <wiringPi.h>

using namespace std;

int wheelSensorGoLowCounter = 1;
double timeDifferenceSeconds = 0.0, milesPerHour = 0.0;
double totalTime;
//clock_t currentTime_1, currentTime_2;
int gpioSpeedometer = 12;
time_t currentTime_1, currentTime_2;
int wiringPiSetupGpio(void);

LIDARLite_v3 myLidarLite;
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
	tmr.setInterval(50L,readSpeedometerSignal); // Call every .05 seconds

	//GPIO.add_event_detect(12, GPIO_FALLING, bouncetime=930); // Testing interrupt
	//if (GPIO.event_detected(12)) {
	//	readSpeedometerSignal();
	//}

	//attachInterrupt(digitalPinToInterrupt(12), readSpeedometerSignal, FALLING);
	//wiringPiISR(12, INT_EDGE_FALLING, &readSpeedometerSignal);
	myLidarLite.i2c_init();     // Initialize i2c peripheral in the cpu core



    myLidarLite.configure(0);    // Optionally configure LIDAR-Lite

}

void loop()
{
    Blynk.run();
    tmr.run();
    //UpdateLidar();
}

/* DECLARE GLOBAL VARIABLES, LIBRARIES AND PIN MODES ABOVE HERE. WRITE FUNCTIONS BELOW */
int count = 0;
void readSpeedometerSignal(){
  if(digitalRead(gpioSpeedometer) == 0){ // Active Low Hall Sensor
	  speedometerFunction();
	  //std::cout << "GPIO PIN is LOW - count: " << count << std::endl;
	  //count++;
	  //delay(928);
    }
}

void speedometerFunction(){
  if(wheelSensorGoLowCounter == 1){
	//currentTime_1 = clock(); // Records time of reading
	  time(&currentTime_1);
	  std::cout << "time 1: " << currentTime_1 << std::endl;
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	wheelSensorGoLowCounter++;
  }
  else if (wheelSensorGoLowCounter > 1 && wheelSensorGoLowCounter < 10) {
	  //currentTime_2 = clock();
	  time(&currentTime_2);
	  //totalTime = (currentTime_2 - currentTime_1); // milliseconds
	  totalTime = difftime(currentTime_2, currentTime_1);
	  std::cout << "Total Time: " << totalTime << std::endl;
	  if (totalTime > 92) { // Debouncing protections
		  std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
		  wheelSensorGoLowCounter++;
	  }
	  else {
		  std::cout << "Debounce error detected, not counting" << endl;
	  }
  }
  else if(wheelSensorGoLowCounter == 10){
	//currentTime_2 = clock();
	  time(&currentTime_2);
	std::cout << "time 2: " << currentTime_2 << std::endl;
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	//totalTime = (currentTime_2 - currentTime_1) * 1e-3; //milliseconds to seconds
	totalTime = difftime(currentTime_2, currentTime_1);
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

void UpdateLidar()
{
	__u16 distance;
	__u8  busyFlag;
	busyFlag = myLidarLite.getBusyFlag();

	if (busyFlag == 0x00)
	{
		myLidarLite.takeRange();
		distance = myLidarLite.readDistance();
		cout << "Distance: " << distance << " cm" << endl;
		Blynk.virtualWrite(V20, (int)distance);
	}
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

