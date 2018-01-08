/*JEEP AUXBUS
 * 
 * Revision 4, 01-01-2018, Initial Public Release
 * 
 * This code is used for an accessory control system in a 2016 Jeep Wrangler Unlimited Sport.  The system  
 * uses an Arduino Uno with a SparkFun Canbus Shield and a custom built proto-shield.  The proto-shield
 * contains power supply components, (2) 8 bit shift registers (74HC595), and an N channel Mosfet (BS170).
 * 
 * This program requires modifications to the standard mcp2515.c file found in the SparkFun Library.  The 
 * modification is needed to filter out un-necessary messages on the CANBUS network.  Failure to filter
 * out un-necessary messages will cause lag in the system as the Arduino is overwhelmed by messages to
 * compare.  As of this release only (3) message IDs are allowed to reach the Arduino (0x208, 0x211, 0x308).
 * All other message IDs are filtered at the CANBUS shield.  The CANBUS shield has 2 masks and 6 filters
 * that can be used.
 * 
 * Reference project schematics for complete system layout.
 * 
 * 
 */


//Include necessary libraries
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <SPI.h>
#include <SoftwareSerial.h>

//define output pins
int latchPin = 19;    //pin used to latch shift register
int clockPin = 5;     //pin used to clock in data on shift register
int dataPin = 4;      //pin used to send data to shift register
int redLED = 9;       //pin used for interior Red LED control

//define input pins
int switch1Pin = 14;  //Analog Pin A0, feedback from switch #1
int switch2Pin = 15;  //Analog Pin A1, feedback from switch #2
int switch3Pin = 16;  //Analog Pin A2, feedback from switch #3
int switch4Pin = 17;  //Analog Pin A3, feedback from switch #4
int switch5Pin = 18;  //Analog Pin A4, feedback from switch #5

//Global Variables
byte outputData1 = 0;   //data for shift register #1
byte outputData2 = 0;   //data for shift register #2
byte speedData = 0;     //speed data from CANBUS
byte lightData = 0;     //light data from CANBUS
byte dimmerData1 = 0;    //dimmer data byte 1 from CANBUS
byte dimmerData0 = 0;   //dimmer data byte 0 from CANBUS
int startCount = 0;     //counter used for system start-up routine
const long pulseTime = 250;   //Pulse time interval (ms)
int pulseCount = 0;     //Pulse counter
const long interval = 250;    //Flash LED interval time (ms)
unsigned long previousMillis = 0;   //variable used to store previous count of clock
unsigned long previousMillis1 = 0;  //variable used to store previous count of clock
int flashState = LOW;   
byte brightness = 0;     //used to store interior LED brightness value 0 to 255
const byte speedLimit = 32;   //used to store speed limit allowable for locker use (32 = 40MPH, 40 = 50MPH)
const byte speedEnable = 5;   //used to store maximum enable speed for lockers (5 = 6 MPH)
/*Speed data from the CANBUS is not a direct speed value, it is 1/2 the KPH.  So to get MPH you take 
* speedData x 2 x 0.621371 = MPH
* This is how 5, 40 and 50 have been chosen as values for speedLimit and speedEnable.
*/


//Define subroutines
void CANinit();
void updateShiftRegister();
void getMessage();
void offroadLights();
void systemStartUp();
void interiorLED();
void lockerRear();
void lockerFront();
void flash();


void setup() {

  //set pin modes
  pinMode(latchPin, OUTPUT);    //Shift Register Latch Pin
  pinMode(dataPin, OUTPUT);     //Shift Register Data Pin
  pinMode(clockPin, OUTPUT);    //Shift Register Clock Pin
  pinMode(redLED, OUTPUT);      //Red Interior LEDs
  pinMode(switch1Pin, INPUT);   //Switch #1 Feedback
  pinMode(switch2Pin, INPUT);   //Switch #2 Feedback
  pinMode(switch3Pin, INPUT);   //Switch #3 Feedback
  pinMode(switch4Pin, INPUT);   //Switch #4 Feedback
  pinMode(switch5Pin, INPUT);   //Switch #5 Feedback

  //set output pin initial values
  digitalWrite(latchPin, HIGH);   //initial value for latch pin on shift registers
  digitalWrite(dataPin, LOW);     //initial value for data pin on shift registers
  digitalWrite(clockPin, LOW);    //initial value for clock pin on shift registers
  analogWrite(redLED, 0);         //initial value for interior LEDs (0 = OFF)

  //set pull-up resistors on input pins
  digitalWrite(switch1Pin, INPUT_PULLUP);  //Enable pullup resistor for Switch #1 Feedback
  digitalWrite(switch2Pin, INPUT_PULLUP);  //Enable pullup resistor for Switch #2 Feedback
  digitalWrite(switch3Pin, INPUT_PULLUP);  //Enable pullup resistor for Switch #3 Feedback
  digitalWrite(switch4Pin, INPUT_PULLUP);  //Enable pullup resistor for Switch #4 Feedback
  digitalWrite(switch5Pin, INPUT_PULLUP);  //Enable pullup resistor for Switch #5 Feedback

  updateShiftRegister();        //Update shift register so all relays are off on startup.
  Canbus.init(CANSPEED_125);    //Initialize MCP2515 CAN controller at 125kbs for interior CANBus
  systemStartUp();              //startup LED sequence

  //Serial.begin(9600);  //for debug use
  
}

void loop() {
  
  getMessage();         //Get CANBUS messages
  offroadLights();      //Update off-road lights
  interiorLED();        //Update interior red LED brightness
  lockerRear();         //Update Rear Locker Control
  lockerFront();        //Update Front Locker Control
  flash();              //trigger LED flash routine
}

void getMessage()   //Get CANBUS Message
{
  tCAN message;     //Poll CANBUS shield to see if there is a message in que
  if (mcp2515_check_message())    //if there is a message then proceed
  {
    if (mcp2515_get_message(&message))  //take message
  {
        if(message.id == 0x211)  //if message is for speed
             {
               speedData = message.data[2]; //transfer hexadecimal speed value to variable speedData
             }
        else if(message.id == 0x208)  //if message is for lighting
             {
               lightData = message.data[0];  //transfer byte 0 to lightData byte
             }
        else if(message.id == 0x308)  //if message has interior light dimmer data
             {
               dimmerData1 = message.data[1];   //transfer byte 1 to dimmerData1
               dimmerData0 = message.data[0];   //transfer byte 2 to dimmerData2
             }
  }}}

void updateShiftRegister()
{
  byte outputFlip = ~outputData1;  //flip bits in "outputData1" because relays are 0 ON, 1 OFF
  digitalWrite(latchPin, LOW);  //Open latch to accept new values
  shiftOut(dataPin, clockPin, MSBFIRST, outputData2);  //Send new Data to shift register #2
  shiftOut(dataPin, clockPin, MSBFIRST, outputFlip);  //Send new Data to shift register #1
  digitalWrite(latchPin, HIGH);  //latch in new values
}

void offroadLights()
{
  byte lightCheck = lightData & B00100000;   //determine if high beam is ON or not, 0 = off, 32 = On
  byte outputCheck = outputData1 & B00000001;  //determine if off-road lights are ON already, 0 = off, 1 = on

  if(digitalRead(switch1Pin) == 0 && lightCheck == 32 && outputCheck == 0)  //TRUE if switch is ON and high beams are ON (32) and output is OFF (0)
  {
    outputData1 = outputData1 | B00000001;  //send ON signal to light output bit
    outputData2 = outputData2 | B00000001;  //send ON signal to switch #1 LED
    updateShiftRegister();  //Update the shift register
  }
  else if((digitalRead(switch1Pin) == 1 || lightCheck != 32) && outputCheck == 1)  //TRUE if high beams are OFF (!=32) and output is ON (1)
  {
    outputData1 = outputData1 & B11111110;  //send OFF signal to light output bit
    outputData2 = outputData2 & B11111110;  //send OFF signal to switch #1 LED
    updateShiftRegister();  //Update the shift register
  }    
}

void systemStartUp()   //used to cycle feedback LEDs during start-up
{
  while(startCount < 3)   //Due this for a total of 3 cycles
  {
    unsigned long currentMillis = millis();   //Used for incrementing LEDs
    if(((currentMillis - previousMillis) >= pulseTime) && (pulseCount < 30))
    {
      pulseCount = (pulseCount * 2) + 1;      //0, 1, 3, 7, 15, 31 repeat
      outputData2 = pulseCount;               //set pulseCount as output to LEDs
      updateShiftRegister();                  //update the shift register
      previousMillis = currentMillis;         //Store latest clock     
    }
    if(pulseCount > 30)               //if interation is done, reset and do another
    {
      delay(250);                     //wait 1/4 sec
      pulseCount = 0;                 //reset pulse count
      outputData2 = pulseCount;       //turn all LEDs off
      startCount = startCount + 1;    //increment iteration counter
      updateShiftRegister();          //update the shift register
      delay(250);                     //wait 1/4 sec
    }
  }
   
}

void interiorLED()    //Used to control interior red LEDs
{
  byte currentDimmer = brightness;    //store current brightness value for comparison
  if(dimmerData0 == 0x13)     //if interior lights are OFF
  {
    brightness = 0;     //set dimmer to OFF
  }

  else if(dimmerData0 == 0x12)      //interior lights are ON
  {
    brightness = 5;     //set dimmer value
  }
  
  if(currentDimmer != brightness)  //if brightness level has changed then update LED brightness
  {
    analogWrite(redLED, brightness);  //Update LED brightness 0 to 255
  }
}

void lockerRear()
{
  byte outputCheck = outputData1 & B00010000;   //Determine if output is ON or OFF
  
  if((digitalRead(switch5Pin) == 0) && (outputCheck != 16))   //Determine if switch is ON and output is OFF
    {
      if(speedData <= speedEnable)   //if speed is less than or equal to enable speed limit turn locker on
      {
        outputData1 = outputData1 | B00010000;    //OR shift register data with bit mask to only turn appropriate bit ON
        outputData2 = outputData2 | B00010000;    //Turn ON switch #5 LED
        updateShiftRegister();  //update the shift register
      }
      else
      {
        if(flashState == LOW)
        {
          outputData2 = outputData2 & B11101111;
        }
        else if(flashState == HIGH)
        {
          outputData2 = outputData2 | B00010000;
        }
        updateShiftRegister();
      }
    }
  if((digitalRead(switch5Pin) == 1) && (outputCheck == 16))  //Determine if switch is OFF and output is ON
    {
      outputData1 = outputData1 & B11101111;    //AND shift register data with bit mask to only turn appropirate bit OFF 
      outputData2 = outputData2 & B11101111;    //Turn OFF switch #5 LED
      updateShiftRegister();  //update the shift register
    }
  if((speedData >= speedLimit) && (outputCheck == 16))  //if speed is greater than or equal to upper speed limit turn locker off
      {
        outputData1 = outputData1 & B11101111;  //AND shift register data with bit mask to only turn appropirate bit OFF
        outputData2 = outputData2 & B11101111;    //Turn OFF switch #5 LED
        updateShiftRegister();  //Update the shift register
      }
   if((digitalRead(switch5Pin) == 1) && ((outputData2 & B00010000) == 16))
   {
    outputData2 = outputData2 & B11101111;
    updateShiftRegister();
   }
}

void lockerFront()
{
  byte outputCheck = outputData1 & B00001000;   //Determine if output is ON or OFF
  
  if((digitalRead(switch4Pin) == 0) && (outputCheck != 8))   //Determine if switch is ON and output is OFF
    {
      if(speedData <= speedEnable)   //if speed is less than or equal to enable speed limit turn locker on
      {
        outputData1 = outputData1 | B00001000;    //OR shift register data with bit mask to only turn appropriate bit ON
        outputData2 = outputData2 | B00001000;    //Turn ON switch #4 LED
        updateShiftRegister();  //update the shift register
      }
      else
      {
        if(flashState == LOW)
        {
          outputData2 = outputData2 & B11110111;
        }
        else if(flashState == HIGH)
        {
          outputData2 = outputData2 | B00001000;
        }
        updateShiftRegister();
      }
    }
  if((digitalRead(switch4Pin) == 1) && (outputCheck == 8))  //Determine if switch is OFF and output is ON
    {
      outputData1 = outputData1 & B11110111;    //AND shift register data with bit mask to only turn appropirate bit OFF 
      outputData2 = outputData2 & B11110111;    //Turn OFF switch #4 LED
      updateShiftRegister();  //update the shift register
    }
  if((speedData >= speedLimit) && (outputCheck == 8))  //if speed is greater than or equal to upper speed limit turn locker off
      {
        outputData1 = outputData1 & B11110111;  //AND shift register data with bit mask to only turn appropirate bit OFF
        outputData2 = outputData2 & B11110111;    //Turn OFF switch #4 LED
        updateShiftRegister();  //Update the shift register
      }
  if((digitalRead(switch4Pin) == 1) && ((outputData2 & B00001000) == 8))
   {
    outputData2 = outputData2 & B11110111;
    updateShiftRegister();
   }
}

void flash()
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis1 >= interval)
  {
    previousMillis1 = currentMillis;
    if(flashState == LOW)
    {
      flashState = HIGH; 
    }
    else {flashState = LOW;}
  }
}


