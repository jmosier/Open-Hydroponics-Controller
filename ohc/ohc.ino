/**
@file ohc.ino
@author OHC Group
*/

#include <time.h>
#include <Wire.h>
//library for RTC
#include <DS3231.h>
//library for DHT sensor
#include <dht.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
//Library for water sensor 
#include <OneWire.h>
#include <DallasTemperature.h>

DS3231 myRTC;
dht DHT;
LiquidCrystal_I2C lcd(0x27,20,4);

int pHSense = A0;
int samples = 10;
float calibrate = 2.12;
float adc_resolution = 1024.0;
#define WaterTempSpread 40   //set to 80 for watertarget
#define TemperatureSpread 40 //set to 80 for temptarget
#define TDSSpread 750  //set tdstarget to 1500
#define pHSpread 1.5    //set pHtarget to 6
//Delete LightTarget and lightspread
#define ONE_WIRE_BUS 10

#define PASSIVE_BUZZER_PIN 9
#define LIGHT_RELAY_PIN 7
#define PUMP_RELAY_PIN 8
#define DHT11_PIN 2 //can be any digital pin (might require PWM, not sure)
// #define LEFT 3   //Left most button (A)
// #define MIDDLE 4 //Middle Button (B)
// #define RIGHT 5 //Right most button (C)
#define FLOAT 6
#define TDS_PIN A1 // TDS pin, can be any analog
#define LIGHT_PIN A2 //can be any analog pinthe cell and 10K pulldown are connected to a0
#define TEMP_PIN A7 //can be any analog pin
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
float voltage;
int water = 0;
int measurings=0;
int photocellReading;     // the analog reading from the sensor divider


#define Left        5 //Left most button (A)
#define Middle      4 //Middle Button (B)
#define Right       3 //Right most button (C)
//Global variables
int screenNumber = 1;
int maintNumber = 0;
bool pressedButton = true;
bool middleButton = false;
bool middleButton2 = false;
//Global Variables for Sensors
double pHLvl = 0;  //Need some changes
double lightLvl = 0;
double humid = 0;
double temp = 0;
float waterTemp = 0;
int error = 0;
#define errorQuantity 1000
//Target Variables
int tempTarget = 80;
int waterTempTarget = 80;
int pHTarget = 6;
int tdsTarget = 1500;
int adjustLights = 0;
int adjustPump = 0;
//Straight Boolean
byte timeArrayPump[] = {B10000000,B00000001,B10000000,B00000001,B10000000,B00000001,B01100000,B00000000,B01100000,B00000000,B00000110,B00000000};
byte timeArrayLights[] = {B00000000,B00000000,B10000000,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B11111111,B00000111,B00000000};
//Water sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/**
  modify a bit at a given position in a byte
  @param num byte to access
  @param position position of bit to access
  @return modified byte value
*/
byte modifyBit(byte num, int position, int value)
{
    int mask = 1 << position;
    return ((num & ~mask) | (value << position));
}

/**
  return a bit at a given position in a byte
  @param num byte to access
  @param position position of bit to access
  @return bit value at position
*/
bool getBit(byte num, int position)
{
	bool bit = num & (1 << position);
	return bit;
}

/**
  Debugging function to print out a 12-length byte array via Serial over USB
  @param timingArray pointer to timing array to print
*/
void printArray(byte *timingArray){
  Serial.println("Timing Array: (O is false, X is true)");
  for(int i = 0; i < 96; i++)
   {
	  int bytenum = i/8;
	  int bitnum = i%8;

     if(getBit(timingArray[bytenum], bitnum))
     {
       Serial.print("X");
     }
     else
     {
       Serial.print("O");
     }
   }
   Serial.print("\n");
   return;
}

/**
  This function activates the passive buzzer and plays a scale
*/
void passiveBuzz(){
  //this functions plays a rapid scale on the passive buzzer as an alarm
  tone(PASSIVE_BUZZER_PIN, 262);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 294);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 330);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 349);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 392);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 440);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 494);
  delay(100);
  tone(PASSIVE_BUZZER_PIN, 523);
  delay(100);
  noTone(PASSIVE_BUZZER_PIN);
}   

/**
  This function reads the current time from the RTC and returns the values through the called pointers
  for some reason, it needs these dummy variables to not throw a hissy fit
  @param Hour pointer to variable to store Hour value
  @param Minute pointer to variable to store Minute value
  @return no return value
*/
void readRTC( int *Hour, int *Minute){
  bool garbageBool2 = false;
  bool garbageBool3 = false;
  *Hour = myRTC.getHour(garbageBool2,garbageBool3);
  *Minute = myRTC.getMinute();
  return;
}

/**
  This function prints the reading from RTC to Serial over USB
*/
void printRTC(){
  int Hour;
  int Minute;
  readRTC(&Hour, &Minute);
  Serial.print("RTC Reading:\n");
  Serial.print("Time:");
  Serial.print(" H ");
  Serial.print(Hour);
  Serial.print(" M ");
  Serial.print(Minute);
  Serial.print("\n");
  return;
}

/**
  Access the DHT (Digital Humidity and Temperature) and store the values of humidity and temperature into the variables at the given pointers
  @param Humidity pointer to variable to store Humidity value
  @param Temperature pointer to variable to store Temperature value
  @return no return value
*/
void readDHT(double *Humidity, double *Temperature){
  //This function reads the current Temperature and Humidity values from the DHT sensor and returns those values through the called pointers (in percentage for Humidity and degrees F for Temperature)
  DHT.read11(DHT11_PIN);
  double humidReading = DHT.humidity;
  if(humidReading >= 100.00)
  {
    humidReading = 99.99;
  }
  if(humidReading < 0)
  {
    humidReading = 0.00;
  }
  *Humidity = humidReading;
 // *Temperature = DHT.temperature;
  *Temperature = (DHT.temperature * 1.80) + 32.00;
  return;
}

/**
  functionDescription
*/
void readWaterTemp(float *WaterTemp){
  sensors.requestTemperatures();
  *WaterTemp = sensors.getTempFByIndex(0);
  return;
}

/**
  This function calls the readDHT function and prints the returned values to the serial USB connection
*/
// void printDHT(){
//   double Humidity;
//   double Temperature;
//   readDHT(&Humidity, &Temperature);
//   Serial.print("DHT Reading:  ");
//   Serial.print("Humidity: ");
//   Serial.print(Humidity);
//   Serial.print(" Temperature: ");
//   Serial.print(Temperature);
//   Serial.print("\n");
//   return;
// }

/**
  AnalogRead the 3-pin temperature probe and store the values into the given pointers
  @param TemperatureC pointer to variable to store Celcius value
  @param TemperatureF pointer to variable to store Fahrenheit value
  @return no return value
*/
// void readTemp(double *TemperatureCptr, double *TemperatureFptr){
//   //This function reads the current Temperature Values (Celcius and Fahrenheit) and returns the values through the called pointers (in degrees)
//   double AbsoluteTemp = analogRead(TEMP_PIN);
//   //the formula below will need tweaking and calibration, but it's probably close enough for now
//   AbsoluteTemp = 124 - (AbsoluteTemp * 0.50);
//   *TemperatureCptr = AbsoluteTemp;
//   *TemperatureFptr = (AbsoluteTemp * 1.80) + 32.00;
//   return;
// }

/**
  This function calls the readTemp function and prints the returned values to the serial USB connection
*/
// void printTemp(){
//   double TemperatureC;
//   double TemperatureF;
//   readTemp(&TemperatureC, &TemperatureF);
//   Serial.print("Temperature Reading:\n");
//   Serial.print("Fahrenheit: ");
//   Serial.print(TemperatureF);
//   Serial.print(" Celcius: ");
//   Serial.print(TemperatureC);
//   Serial.print("\n");
//   return;
// }

/**
  This function reads the current light level from the Photoresistor and returns the value through the called pointers (as a percentage)
  @param LightLevel pointer to variable to store LightLevel value
  @return no return value
*/
void readLight(double *LightLevel){
  // const int photoresistorPin = A2;  // Define the pin for the photoresistor
  const float pullupResistance = 10000.0; // Define the resistance of the pull-up resistor
  const float quantumValue = 2.6; //quantum value of Samsung LM301B LED's ( in umol/J)
  const float conversionFactor = 0.0079; // Define the conversion factor from umol/m2/s to lux

  const float calibrationValue = 11.0; // Replace this value with your calibrated PPFD value Defined: (in umol/m2/s)
  float photoresistorVoltage = analogRead(LIGHT_PIN);
  //Serial.println(analogRead(LIGHT_PIN));
  
  float photoresistorResistance = (VREF / photoresistorVoltage - 1) * pullupResistance;  // Convert the voltage to resistance using the pull-up resistor
  float ppfd = (750000)*(photoresistorVoltage * quantumValue / ((photoresistorResistance * photoresistorResistance)/5));  // (WIP)Calculate the PPFD value in umol/m2/s using the formula for a photoresistor
//ppfd is currently calculating the percentage of light

  // float calibrationFactor = calibrationValue / ppfd;  // Calculate the calibration factor
  // ppfd *= calibrationFactor;  // Apply the calibration factor to the PPFD value
  float lux = ppfd * conversionFactor;  // Convert PPFD to lux using the conversion factor
  *LightLevel= ppfd;

  // double AbsoluteLight = analogRead(LIGHT_PIN);
  // //just doing a percentage for now, we can decide on a unit (if we want one) later
  // AbsoluteLight = AbsoluteLight / 1024.00;
  // *LightLevel = 100.00 - (AbsoluteLight * 100.00);
  return;
}

/**
  This function calls the readLight function and prints the returned value to the serial USB connection
*/
void printLight(){
  double LightLevel;
  readLight(&LightLevel);
  Serial.print("Light% Reading:\n");
  Serial.print("Light level: ");
  Serial.print(LightLevel);
  Serial.print("%\n");
  delay(1000);
  return;
}

/**
  read TDS through the analogRead and store value into (WIP)
*/
void readTDS(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN); //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT)
    analogBufferIndex = 0;
  }
  for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
    analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient=1.0+0.02*(((waterTemp-32.0)*(5.0/9.0))-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVoltage=averageVoltage/compensationCoefficient; //temperature compensation
    tdsValue=(4.0/3.0)*((133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5); //convert voltage value to tds value
  return;
}

/**
  This function calls the readTDS function and prints the returned value to the serial USB connection
*/
void printTDS(){
  readTDS();
  Serial.print("TDS Value: ");
  Serial.print(tdsValue);
  Serial.print(" ");
  Serial.println("ppm");
  return;
}

/**
  functionDescription
  @param bArray[] paramDescription
  @param iFilterLen paramDescription
  @return returnValueDescription
*/
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
  else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

/**
  functionDescription (WIP)
*/
void readPH(){
  measurings=0;
    for (int i = 0; i < samples; i++)
    {
        measurings += analogRead(pHSense);
        delay(10);
    }

    voltage = 5.0 / adc_resolution * measurings/samples;
    pHLvl = (7 + ((2.5 - voltage) / 0.18)) + calibrate;
    // for (int i = 0; i < samples; i++)
    // {
    //     measurings += analogRead(pHSense);
    //     delay(10);
    // }
  return;
}

/**
  This function calls the readPH function and prints the returned value to the serial USB connection
*/
void printPH(){
  readPH();
  Serial.print("pH= ");
  Serial.println(pHLvl);
  //Serial.println((7 + ((2.5 - voltage) / 0.18)) + calibrate);
  // voltage = 5 / adc_resolution * measurings/samples;
  // Serial.print("pH= ");
  // Serial.println(ph(voltage));
  return;
}

/**
  read digital input (HI or LO) of water level float sensor and store into the variable at the given pointer
*/
void readWater(){
  //update to pointer notation
    water = digitalRead(FLOAT);   // read the input pin
  return;
}

/**
  This function calls the readWater function and prints the returned value to the serial USB connection
*/
// void printWater(){
//   readWater();
//   Serial.print("float= ");
//   Serial.println(water);
//   return;
// }

/**
  Triggers the LED lights relay closed when true and open when false
  @param state 0 or 1 for setting lights off or on
*/
void setLightRelay(int state){
  if(state == 1){
    digitalWrite(LIGHT_RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(LIGHT_RELAY_PIN, LOW);
  }
}

/**
  Triggers the pump relay closed when true and open when false
  @param state 0 or 1 for setting lights off or on
*/
void setPumpRelay(int state){
  if(state == 1){
    digitalWrite(PUMP_RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(PUMP_RELAY_PIN, LOW);
  }
}

/**
  functionDescription (WIP)
*/
void clockCompare(){
  int hoursCompare;
  int minutesCompare;
  readRTC(&hoursCompare, &minutesCompare);
   int time = 0;
   time = (hoursCompare*60) + minutesCompare;
   time = time/15;
   if(timeArrayPump[time] == true)
   {
     adjustPump = 1;
   }
   else
   {
     adjustPump = 0;
   }
   if(timeArrayLights[time] == true)
   {
     adjustLights = 1;
   }
   else 
   {
     adjustLights = 0;
   }
  return;
}

/**
  UI for entering the start of a time period for Lights or Pumps, depending on boolean
  @param settingTime pointer to current settingTime value
  @param LightOrPump 1 for adjusting Lights, 0 for adjusting Pump
  @return returns true if user requests exiting and saving changes, false if not
*/
bool setStartTime(int *settingTime, bool LightOrPump){

   lcd.clear();
   while(digitalRead(Middle)==HIGH)
   {
    int displayHours = *settingTime/4;
    int displayMinute = (*settingTime%4)*15;
 
    /**Serial.print(displayHours);
    Serial.print(":");
    Serial.print(displayMinute);
    Serial.print("\n");*/
    
    lcd.setCursor(0,0);
	if(LightOrPump)
	{
		lcd.print("Start of Light ON");
	}
	else
	{
		lcd.print("Start of Pump ON");
	}
    lcd.setCursor(0,1);
    lcd.print("Time:");
    if(displayHours > 9)
    {
      lcd.setCursor(0,2);
    }
    else
    {
      lcd.setCursor(0,2);
      lcd.print(" ");
      lcd.setCursor(1,2);
    }

    lcd.print(displayHours);
    lcd.setCursor(3, 2);
    lcd.print(":");
    if(displayMinute > 9)
    {
      lcd.setCursor(4,2);
    }
    else
    {
      lcd.setCursor(4,2);
      lcd.print("0");
      lcd.setCursor(5,2);
    }
    lcd.print(displayMinute);  
    lcd.setCursor(12,2);
    lcd.print("GO ABOVE");
    lcd.setCursor(7,3);
    lcd.print("23:45 TO EXIT");
    lcd.display();
    if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
      {
            //  pressedButton = true;
        delay(100);
         *settingTime = *settingTime - 1;
             
      }
      else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
         {
            //  pressedButton = true;
          delay(100);
           *settingTime = *settingTime + 1;
         }
      if(*settingTime < 0)
      {
        *settingTime = 95;
      }
      if(*settingTime >= 96)
      {
        bool leaveMeAlone = false;
        lcd.clear();
         while(leaveMeAlone == false)
         {
           
           lcd.setCursor(0,0);
           lcd.print("Done Adjusting?");
           lcd.setCursor(0,2);
           lcd.print("NO               YES");
           lcd.setCursor(0,3);
           lcd.print("<--              -->");
           lcd.display();

           if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
             {
            //  pressedButton = true;
              leaveMeAlone = true;
              *settingTime = 95;  
               delay(100);            
             }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
             {
                //  pressedButton = true;
                delay(100);
                screenNumber++;
                return true;         
             }
           }
          delay(100);
      }
    delay(100);
	//Serial.println(*settingTime);
   }
   
   return false;
}

/**
  UI for entering the end of a time period for Lights or Pumps, depending on boolean
  @param settingTime pointer to current settingTime value
  @param timingArray pointer to timing array to adjust
  @param LightOrPump 1 for adjusting Lights, 0 for adjusting Pump
*/
void setEndTime(int *settingTime, byte *timingArray, bool LightOrPump){
   int startSettingTime = *settingTime;
   *settingTime = *settingTime + 1;
   lcd.clear();
   while(digitalRead(Middle)==HIGH)
   {
    int displayHours = *settingTime/4;
    int displayMinute = (*settingTime%4)*15;
    
    /**Serial.print(displayHours);
    Serial.print(":");
    Serial.print(displayMinute);
    Serial.print("\n");*/

    lcd.setCursor(0,0);
	if(LightOrPump)
	{
		lcd.print("End of Light ON");
	}
	else
	{
		lcd.print("End of Pump ON");
	}
    lcd.setCursor(0,1);
    lcd.print("Time:");
    if(displayHours > 9)
    {
      lcd.setCursor(0,2);
    }
    else
    {
      lcd.setCursor(0,2);
      lcd.print(" ");
      lcd.setCursor(1,2);
    }

    lcd.print(displayHours);
    lcd.setCursor(3, 2);
    lcd.print(":");
    if(displayMinute > 9)
    {
      lcd.setCursor(4,2);
    }
    else
    {
      lcd.setCursor(4,2);
      lcd.print("0");
      lcd.setCursor(5,2);
    }
    lcd.print(displayMinute);  
    lcd.display();
    if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
      {
            //  pressedButton = true;
        delay(100);
         *settingTime = *settingTime - 1;
             
      }
      else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
         {
            //  pressedButton = true;
          delay(100);
           *settingTime = *settingTime + 1;
              
         }
      if(*settingTime < 0)
      {
        *settingTime = 96;
      }
      if(*settingTime > 96)
      {
        *settingTime = 0;
      }
    delay(100);
    //Serial.println(*settingTime);
   }
   for(int i = startSettingTime; i < *settingTime;i++)
   {
	   int bytenum = i/8;
	   int bitnum = i%8;
	   timingArray[bytenum] = modifyBit(timingArray[bytenum], bitnum, 1);
   }
   return;
}

/**
  allows user to enter the time periods for Lights or Pumps to be on, depending on boolean
  @param timingArray pointer to light or pump timing array to adjust
  @param LightOrPump 1 for adjusting Lights, 0 for adjusting Pump
*/
void adjustTimingArray(byte *timingArray, bool LightOrPump){
          bool leaveMeAlone = false;
        lcd.clear();
         while(leaveMeAlone == false)
         {
         delay(100);
           lcd.setCursor(0,0);
           lcd.print("Do you want to");
           lcd.setCursor(0,1);
           if(LightOrPump)
		   {
			   lcd.print("Adjust Light Times?");
		   }
			else
			{
				lcd.print("Adjust Pump Times?");
			}
           lcd.setCursor(0,2);
           lcd.print("NO               YES");
           lcd.setCursor(0,3);
           lcd.print("<--              -->");
           lcd.display();

           if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
             {
			   lcd.clear();
               pressedButton = true;
               screenNumber++;
               leaveMeAlone = true;
			   delay(100);
               return;           
             }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
             {
                //pressedButton = true;
                delay(100);
                leaveMeAlone = true;         
             }
           }
    for(int i = 0; i<12; i++)
    {
       timingArray[i] = B00000000;
    }
    int settingTime = 0;

    bool exitcondition = false;
    while(exitcondition == false)
    {
        exitcondition = setStartTime(&settingTime, LightOrPump);
        if(exitcondition == false)
        {
          delay(200); //maybe reduce to 100
          setEndTime(&settingTime, timingArray, LightOrPump);
          delay(200);
        }
    }
	//lcd.clear();
    return; 
}

/**
  functionDescription
  @param sensorValue paramDescription
  @param units paramDescription
  @param sensorName paramDescription
  @param lastNextScreen paramDescription
*/
void maintDisplay(int sensorValue, String units, String sensorName, String lastNextScreen){
    lcd.setCursor(0,0);
    lcd.print(sensorName);
    lcd.setCursor(6,1);
    lcd.print(sensorValue);
    lcd.setCursor(11,1);
    lcd.print(units);
    lcd.setCursor(0, 2);
    lcd.print("Config w/ MidButton");   
    lcd.setCursor(0, 3);
    lcd.print(lastNextScreen);  
    lcd.display(); 
    return; 
}

/**
  functionDescription
  @param sensorValue paramDescription
  @param units paramDescription
  @param sensorName paramDescription
  @param lastNextScreen paramDescription
*/
void screenDisplay(float sensorValue, String units, String sensorName, String lastNextScreen){ //Function for formatting screens
    lcd.setCursor(0,0);
    lcd.print(sensorName);
    lcd.setCursor(6,1);
    lcd.print(sensorValue);
    lcd.setCursor(12,1);
    lcd.print(units);   
    lcd.setCursor(0, 3);
    lcd.print(lastNextScreen);  
    lcd.display();
    return;
}

/**
  functionDescription
*/
void updateValues(){
   readDHT(&humid,&temp);
   readWaterTemp(&waterTemp);  
   readWater();
   readLight(&lightLvl);
   readTDS();
   readPH();
}

/**
  void statement to check if all variables are within spread
*/
bool spreadChecker (){
  bool good = true;
  if(abs(temp-tempTarget) > TemperatureSpread)
  {
    good = false;
    if(error >= errorQuantity)
    {
      screenNumber = 2;
      pressedButton = true;   
      lcd.setCursor(0,2);
      lcd.print("Temp out of range!");
    }
    

  }
  if(abs(waterTemp-waterTempTarget) > WaterTempSpread)
  {
    good = false;
    if(error >= errorQuantity)
    {
    screenNumber = 2;
    pressedButton = true;  
    lcd.setCursor(0,2);
    lcd.print("WaterTemp out range!");
    }
  }
  if(abs(tdsValue-tdsTarget) > TDSSpread)
  {
    good = false;
    if(error >= errorQuantity)
    {    
    screenNumber = 4;
    lcd.setCursor(0,2);
    lcd.print("TDS out of range!");
    }
  }
  if(abs(pHLvl-pHTarget) > pHSpread)
  {
    good = false;
    if(error >= errorQuantity)
    {
    screenNumber = 6;
    pressedButton = true;
    lcd.setCursor(0,2);
    lcd.print("pHLvl out of range!");
  }
  if(water == 0)
  {
    good = false;
    if(error >= errorQuantity)
    {

    screenNumber = 5;
    pressedButton = true;
    lcd.setCursor(0,2);
    lcd.print("No Water Detected!");
    }
  }  
  
  return good;
}
}

/**
  functionDescription
*/
void updateScreen(){
 
  if(pressedButton == true)
  { //Checking if button has been pressed
	lcd.clear();
	pressedButton = false;
  }
 
    switch(screenNumber)
    {
      case 1:
        lcd.setCursor(6,1);
        lcd.print("Monitor");
        lcd.setCursor(6,2);
        lcd.print("Screens");
        lcd.display();
        maintNumber = 0;
      break;
      
      case 2: //Temp sensor
        readDHT(&humid,&temp);
        readWaterTemp(&waterTemp);
        screenDisplay(temp, "F", "Temperature:", "<-MonitorSCR Humid->"); //Calling screenDisplay and making screen for variable
        lcd.setCursor(0,2);
        lcd.print("Water Temp:"); 
        lcd.setCursor(12,2);
        lcd.print(waterTemp);
        lcd.setCursor(17,2);   
        lcd.print("F");                   
        maintNumber = 0;
      break;
  
      case 3: //Humidity Sensor
        readDHT(&humid,&temp);
        screenDisplay(humid, "%", "Humidity:", "<- Temp       TDS ->");
        maintNumber = 0;
      break;

      case 4: //TDS sensor
        readTDS();
        screenDisplay(tdsValue, "ppm", "TDS:", "<- Humid  WaterLVL->");
        maintNumber = 0;
      break;

      case 5: //Water Level Sensor  
        readWater();      //Calling water sensor
        lcd.setCursor(0,0);
        lcd.print("Water Present? 1=Yes");
        lcd.setCursor(7,1);
        lcd.print(water);
        lcd.setCursor(11,1);
        lcd.print("    0=No");   
        lcd.setCursor(0, 3);
        lcd.print("<- TDS         pH ->");  
        lcd.display();
        maintNumber = 0;
      break;

      case 6: //pH sensor
       readPH();
       screenDisplay(pHLvl, "pH", "pH:", "<-WaterLVL  Light%->");
       maintNumber = 0;
      break;

      case 7:// Light% 
        readLight(&lightLvl);
        screenDisplay(lightLvl, "%", "Light Level:", "<- pH  MonitorSCR ->");
        maintNumber = 0;
      break;

      // case 8: //SD  Card
      //   screenDisplay(tdsValue, "    TDS=    ppm", "WaterLVL->", "<- Humid");  
      //   maintNumber = 0;
      // break;

      case 8:
        lcd.setCursor(6,1);
        lcd.print("Monitor");
        lcd.setCursor(6,2);
        lcd.print("Screens");
        lcd.display();
        maintNumber = 0;
      break;

    //Do  not quite know what to put for SD Card or adjust light screen and pump screen or what value is needed to adjust so just kept them blank 
    //Maintenance Screens Below
      case 9:
        lcd.setCursor(4,1);
        lcd.print("Maintenance");
        lcd.setCursor(6,2);
        lcd.print("Screens");
        lcd.display();
        maintNumber = 0;
      break;

      case 10: //Adjust Light Screen
        maintDisplay(adjustLights, "    0=OFF", "Lights:        1=ON", "<-MaintSCR LGTTime->");
        maintNumber = 1;    
      break;
      case 11: //Adjust Light Array Screen
        //printArray(timeArrayLights);
        adjustTimingArray(timeArrayLights, true);
        //printArray(timeArrayLights);
      break;

      case 12: //Adjust Pump Screen
        maintDisplay(adjustPump, "    0=OFF", "Pump:          1=ON", "<-LGTTime  PMPTime->"); //Calling screenDisplay and making screen for variable
        maintNumber = 2; 
      break;
      case 13: //Adjust Pump Array Screen
		adjustTimingArray(timeArrayPump, false);
      break;
  
      case 14: //Temp sensor maintenance
        maintDisplay(tempTarget, "F", "Target Temperature:", "<-PMPTime WTRTemp->");
        maintNumber = 3; 
      break;

      case 15: //WaterTemp Sensor maintenance
        maintDisplay(waterTempTarget, "%", "Target Water Temp:", "<-TempSens TDSSens->");
        maintNumber = 4; 
      break;

      case 16:  //TDS sensor maintenance
        maintDisplay(tdsTarget, "ppm", "Target TDS:", "<-WTRTemp   pHSens->");
        maintNumber = 5; 
      break;

      case 17: //pH sensor maintenance
        maintDisplay(pHTarget, "pH", "Target pH:", "<-TDSSens  Light %->");
        maintNumber = 6; 
      break;

      case 18:// Maintenance Title
          lcd.setCursor(4,1);
          lcd.print("Maintenance");
          lcd.setCursor(6,2);
          lcd.print("Screens");
          lcd.display();
          maintNumber = 0;
      break;


      default:
      break;
  }
  
  delay(100);
  clockCompare();
  setPumpRelay(adjustPump);
  setLightRelay(adjustLights);
  updateValues();
  if(spreadChecker() == false)
  {
    //error++;
    //Serial.println(error);
    if(error >= errorQuantity)
    {

    while(digitalRead(Middle)==HIGH)
    {
      passiveBuzz(); //Alarm Functions if values are not in spread
      delay(1000);
    }
    error = 0;
    }
  }


  //Changing Values for monitoring Screen Values
  if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
  {
    pressedButton = true;
    delay(100);
    screenNumber = screenNumber - 1;
    if(screenNumber <1)
    {
      screenNumber = 18;
    }
  }
  else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
  {
    pressedButton = true;
    delay(100);
    screenNumber++;
    if(screenNumber > 18)
    {
      screenNumber = 1;
    }
  }
  //Going Into Maintenance screens
  else if(digitalRead(Middle)==LOW)
  {
    // pressedButton = true;
    // screenNumber = 8;
    middleButton2 = true;

    delay(100);
  }
  while(middleButton2 == true) //Check for if Middle button is pressed then configure values
    {
     // pressedButton = true;
      delay(100);
            //  Serial.println("I am inside While loop 2"); //Test to ensure you are in while loop
      switch(maintNumber) //maintNumber set in case files from screens
      {
        case 0:
          if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
          }       
        break;
        case 1:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
              //pressedButton = true;
              delay(100);
              adjustLights = adjustLights - 1;
              if(adjustLights > 1)
              {
                adjustLights = 0;
              }
              else if(adjustLights < 0)
              {
                adjustLights = 1;
              }              
              
              maintDisplay(adjustLights, "    0=OFF", "Lights:        1=ON", "<- MaintSC ADJPump->");
              //  Serial.println("I have decrease the lights by 1"); //Test to ensure you are adjusting values
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              adjustLights++;
              if(adjustLights > 1)
              {
                adjustLights = 0;
              }
              else if(adjustLights < 0)
              {
                adjustLights = 1;
              }               
              maintDisplay(adjustLights, "    0=OFF", "Lights:        1=ON", "<- MaintSC ADJPump->");    
             //  Serial.println("I have increased the lights by 1"); //Test to ensure you are adjusting values
            }
          else if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
            pressedButton = true;
          }
        break;
        case 2:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              adjustPump = adjustPump - 1;
              if(adjustPump > 1)
              {
                adjustPump = 0;
              }
              else if(adjustPump < 0)
              {
                adjustPump = 1;
              }               
              maintDisplay(adjustPump, "    0=OFF", "Pump:          1=ON", "<-ADJLGT TempSens->");           
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              adjustPump++;
              if(adjustPump > 1)
              {
                adjustPump = 0;
              }
              else if(adjustPump < 0)
              {
                adjustPump = 1;
              }   
              maintDisplay(adjustPump, "    0=OFF", "Pump:          1=ON", "<-ADJLGT TempSens->");
            }
          else if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
            pressedButton = true;
          }          
        break;
        case 3:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              tempTarget = tempTarget - 1;
              maintDisplay(tempTarget, "F", "Target Temperature:", "<-ADJPump  WTRTemp->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              tempTarget++;
              maintDisplay(tempTarget, "F", "Target Temperature:", "<-ADJPump  WTRTemp->");
            }
          else if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
            pressedButton = true;
          }
        break;
        case 4:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              waterTempTarget = waterTempTarget - 1;
              maintDisplay(waterTempTarget, "%", "Target Water Temp:", "<-TempSens TDSSens->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              waterTempTarget++;
              maintDisplay(waterTempTarget, "%", "Target Water Temp:", "<-TempSens TDSSens->");
            }
          else if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
            pressedButton = true;
          }
        break;
        case 5:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              tdsTarget = tdsTarget - 1;
              maintDisplay(tdsTarget, "ppm", "Target TDS:", "<-WTRTemp WaterLVL->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              tdsTarget++;
              maintDisplay(tdsTarget, "ppm", "Target TDS:", "<-WTRTemp WaterLVL->");
            }
          else if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
            pressedButton = true;
          }
        break;
        // case 6:
        //   if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
        //     {
        //    //   pressedButton = true;
        //       delay(100);
        //       waterLvl = waterLvl - 1;
        //     }
        //   else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
        //     {
        //     //  pressedButton = true;
        //       delay(100);
        //       waterLvl++;
        //     }
        //   else if(digitalRead(Middle)==LOW)
        //   {
        //     delay(100);
        //     // middleButton = false;
        //     middleButton2 = false;
        //     pressedButton = true;
        //   }
        // break; 
        case 6:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              pHTarget = pHTarget - 1;
              maintDisplay(pHTarget, "pH", "Target pH:", "<-WaterLVL Light %->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              pHTarget++;
              maintDisplay(pHTarget, "pH", "Target pH:", "<-WaterLVL Light %->");
            }
          else if(digitalRead(Middle)==LOW)
          {
            delay(100);
            // middleButton = false;
            middleButton2 = false;
            pressedButton = true;
          }
        break;   
  
        default:
        break;     
      }
      // middleButton2 = false;
    }

  delay(25);
  //Serial.println(screenNumber);
  // Serial.println(middleButton);
  // Serial.println(middleButton2);
}
void printSensorValues(){  //void function for debugging by seeing all sensor values in Serial Monitor and on LCD
   //reading all sensors
    updateValues();
    Serial.print(temp);
    Serial.print("   ");
    Serial.print(waterTemp);
    Serial.print("   ");
    Serial.print(humid);
    Serial.print("   ");
    Serial.println(tdsValue);
    Serial.print(water);
    Serial.print("   ");
    Serial.print(pHLvl);
    Serial.print("   ");
    Serial.println(lightLvl);
    Serial.print("/n");
    //LCD value prints
    lcd.setCursor(0, 0);
    lcd.print(temp);
    lcd.setCursor(5, 0);
    lcd.print("F");
    lcd.setCursor(7, 0);
    lcd.print(waterTemp);
    lcd.setCursor(12, 0);
    lcd.print("F");
    lcd.setCursor(14, 0);
    lcd.print(humid);
    lcd.setCursor(19, 0);
    lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print(tdsValue);
    lcd.setCursor(7, 1);
    lcd.print("ppm");
    lcd.setCursor(11, 1);
    lcd.print(water);
    lcd.setCursor(13, 1);
    lcd.print("water");
    lcd.setCursor(0, 2);
    lcd.print(pHLvl);
    lcd.setCursor(5, 2);
    lcd.print("pH");
    lcd.setCursor(11, 2);
    lcd.print(lightLvl);
    lcd.setCursor(17, 2);
    lcd.print("%");
    return;
}
/**
  function launches on board startup
*/
void setup() {
  Wire.begin();
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(TDS_PIN,INPUT);
  pinMode(LIGHT_PIN,INPUT);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(PASSIVE_BUZZER_PIN, OUTPUT);
  // pinMode(LEFT, INPUT);
  // pinMode(MIDDLE, INPUT);
  // pinMode(RIGHT, INPUT);
  pinMode(FLOAT, INPUT);
  sensors.begin();
  myRTC.setClockMode(false); //sets to 24H format
  //uncomment the following lines and change values accordingly to set the time on the RTC:
  /**
   myRTC.setYear(23);
   myRTC.setMonth(1);
   myRTC.setDate(25);
   myRTC.setHour(4);
   myRTC.setMinute(25);
   myRTC.setSecond(0);
   **/
}

/**
  function loops forever
*/
void loop() {
  //printLight();
  // Serial.print("\n\n");
  // Serial.print("---------------------------\n");
  // printRTC();
  // printDHT();
 // printLight();
  // printTDS();
  // printPH();
  // printWater();
  // printSensorValues();
  updateScreen();
  
}
