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
void wheelRevolutionFunction();
void speedometerReadingCalculation(double totalTime);
void turnOnLeftTurnSignal();
void turnOnRightTurnSignal();
void UpdateLidar();
void initializeAdc();
void initializeMotor();
void changeSpeed(int dutyCycle);
void configureMotorDriver();
void convertToEight(unsigned char eightBits[],unsigned short sixteenBits);
void brake();
void changeSpeed();
#endif

WidgetLED led1(V16);
WidgetLED led2(V19);

#include <time.h> /* Will be used for MPH */
#include <stdio.h> 
#include <cmath>
#include <iomanip>      
#include <linux/types.h>
#include <cstdio>
#include <lidarlite_v3.h>
#include <iostream>
#include <fstream>
#include <chrono>	
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>//FOR ADC

using namespace std;

int wheelSensorGoLowCounter = 1;
double timeDifferenceSeconds = 0.0, milesPerHour = 0.0;
double totalTime;
std::chrono::time_point<std::chrono::high_resolution_clock> currentTime_1, currentTime_2; // Chrono timing libraries, high resolution clock
std::chrono::duration<double> totalDuration;

int gpioSpeedometer = 12; // General Purpose Input Output Pins
int gpioRightTurnSignal = 16;
int gpioLeftTurnSignal = 19;

LIDARLite_v3 myLidarLite;
BlynkTimer tmr;
//Motor Driver Register Values Based on DRV8320 Register Table
#define DRIVERCONTROL       0b1001000001000010
#define GATEDRIVEHSREGISTER 0b1001101111111111
#define GATEDRIVELSREGISTER 0b1010011111111111
#define OCPCONTROLREGISTER  0b1010100101011001
//PINS
int gpioMotorPwm=24;
int gpioMotorBrake=25;
int gpioMotorEnable=26;
//SPI virtual PIN where ADC value is stored
int adcVirtualPin=100;

int motorPwmDutyCycle=0;
int motorChannel=1;//SPI channel 1
int adcChannel=0;//SPI Channel 0
int analogThrottleValue=0;
char spiIn[2];
int wetModeReduction=0;
unsigned char spiOut[2];
bool braking=false;
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
	pinMode(16, INPUT); // GPIO 16, pin 36
	pinMode(19, INPUT); // GPIO 19, pin 35
	//tmr.setInterval(50L,readSpeedometerSignal); // Call every .05 seconds

	//GPIO.add_event_detect(12, GPIO_FALLING, bouncetime=930); // Testing interrupt
	//if (GPIO.event_detected(12)) {
	//	readSpeedometerSignal();
	//}

	//attachInterrupt(digitalPinToInterrupt(12), readSpeedometerSignal, FALLING);
	wiringPiISR(12, INT_EDGE_FALLING, &readSpeedometerSignal); // Call readSpeedometerSignal
	wiringPiISR(16, INT_EDGE_BOTH, &turnOnRightTurnSignal); // Call turnOnRightTurnSignal
	wiringPiISR(19, INT_EDGE_BOTH, &turnOnLeftTurnSignal); // Call turnOnLeftTurnSignal

	myLidarLite.i2c_init();     // Initialize i2c peripheral in the cpu core
    myLidarLite.configure(0);    // Optionally configure LIDAR-Lite
    
    //Motor/Throttle Initialization 
    initializeMotor();
    initializeAdc();
    configureMotorDriver();
    
    

}

void loop()
{
    Blynk.run();
    tmr.run();
    changeSpeed();
    //UpdateLidar();
}

/* DECLARE GLOBAL VARIABLES, LIBRARIES AND PIN MODES ABOVE HERE. WRITE FUNCTIONS BELOW */
int count = 0;
void readSpeedometerSignal(){
  if(digitalRead(gpioSpeedometer) == 0){ // Active Low Hall Sensor
	  wheelRevolutionFunction();
	  //std::cout << "GPIO PIN is LOW - count: " << count << std::endl;
	  //count++;
	  //delay(928);
    }
}

void wheelRevolutionFunction(){
  if(wheelSensorGoLowCounter == 1){ // Check counter if first measurement
	currentTime_1 = std::chrono::high_resolution_clock::now(); // Store time 1
	wheelSensorGoLowCounter++;
  }
  else if (wheelSensorGoLowCounter > 1 && wheelSensorGoLowCounter < 10) { // Check counter if measurement #2-9
	  currentTime_2 = std::chrono::high_resolution_clock::now(); // Store time 2
	  totalDuration = currentTime_2 - currentTime_1; // Time2-Time1=Duration
	  totalTime = std::chrono::duration<double>(totalDuration).count(); // Convert to seconds and type double
	  cout << "Duration Time : " << totalTime << endl;
	  if (totalTime > .0927) { // Debouncing protections (92.7ms)
		  std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
		  wheelSensorGoLowCounter++;
	  }
	  else {
		  std::cout << "Debounce error detected, not counting" << endl;
	  }
  }
  else if(wheelSensorGoLowCounter == 10){ // Check if final measurement
	currentTime_2 = std::chrono::high_resolution_clock::now(); // Store time 2
	totalDuration = currentTime_2 - currentTime_1; // Time2-Time1=Duration
	totalTime = std::chrono::duration<double>(totalDuration).count(); // Total time from first to last measurement
	std::cout << "wheelSensorGoLow:" << wheelSensorGoLowCounter << std::endl;
	std::cout << "timeDifferenceSeconds: " << totalTime << std::endl;
    speedometerReadingCalculation(totalTime); // Pass totalTime to MPH function
	std::cout << "MPH:" << milesPerHour << std::endl;
    Blynk.virtualWrite(V12,milesPerHour); // Write value to gauge on Blynk
	wheelSensorGoLowCounter = 1; // Reset counter
  }
  else{}
}

void speedometerReadingCalculation(double totalTime){
	milesPerHour = (5* 2 * M_PI*(1.083) * 60 * 60) / (5280 * totalTime);
}

void turnOnRightTurnSignal() {
	if (digitalRead(gpioRightTurnSignal) == 0) {
		led1.off();
	}
	else {
		led1.on();
	}
}

void turnOnLeftTurnSignal() {
	if (digitalRead(gpioLeftTurnSignal) == 0) {
		led2.off();
	}
	else {
		led2.on();
	}
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

void convertToEight(unsigned char eightBits[],unsigned short sixteenBits){
  //Converts 16 bits to 2 8 bits
  eightBits[0]=sixteenBits>>8;
  eightBits[1]=sixteenBits & 0x00FF;
}
void configureMotorDriver(){
  //Configures the SPI Channel
  wiringPiSPISetup(motorChannel,1000000);
  unsigned short config[]={DRIVERCONTROL,GATEDRIVEHSREGISTER,
                           GATEDRIVELSREGISTER,OCPCONTROLREGISTER};
  for (int i=0;i<4;i++){
    //Sends the Register Values to the Motor Driver Chip
    convertToEight(spiOut,config[i]);
    wiringPiSPIDataRW(motorChannel,spiOut,2);
  }
}
void initializeMotor(){
  //Assigns all the pins for Motor Driver
  pinMode(gpioMotorBrake,OUTPUT);//Pin for braking 
  digitalWrite(gpioMotorBrake,1);//Initializes 1 to signal no-braking
  pinMode(gpioMotorPwm,PWM_OUTPUT);//Pin to Send PWM signal
  pwmWrite(gpioMotorPwm,motorPwmDutyCycle);//Sends current Duty Cycle
}
void initializeAdc(){
  //Sets Up the ADC
  mcp3004Setup(adcVirtualPin,adcChannel);
}
BLYNK_WRITE(V25){
  int pinValue=param.asInt();//Read Pin Value
  if(pinValue==0) wetModeReduction=0;//Sets to Dry Mode
  else wetModeReduction=1;//Sets to wet Mode (cause speed Reduction)
}
void changeSpeed(){
  //Change the speed by sending the current PWM duty cycle
  motorPwmDutyCycle=analogRead(adcVirtualPin);
  //Reduce to 60% if wet Mode is on
  motorPwmDutyCycle-=(wetModeReduction*(int(motorPwmDutyCycle*0.4)));
  pwmWrite(gpioMotorPwm,motorPwmDutyCycle);
}
void brake(){
  //
  if(!braking) {//If it is already breaking then disbale braking
     digitalWrite(gpioMotorBrake,1);
     braking=true;
   }
  else{//Else brake
      digitalWrite(gpioMotorBrake,0);
      braking=false;
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

