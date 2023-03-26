//Sean Test Push 3/26/2023
//test2

//Jonathan Kent
//Changes Added: added dummy spread values, added target values, changed maintenance screens and functions to use or change target values
// added spreadChecker to check spread values, called readSensor() statements to monitor screen sections to get sensor values
//Added buzzer, 
//Need to add read functions for tds and ph
//Need to add 4 screens, pump time on and off, light time on and off, and compare clock with when to turn on/off pumps and lights

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
#define HumiditySpread 10
#define TemperatureSpread 10
#define TDSSpread 100
#define pHSpread 1
#define ONE_WIRE_BUS 10
#define LightLevelSpread 10
#define PASSIVE_BUZZER_PIN 9
#define LIGHT_RELAY_PIN 7
#define PUMP_RELAY_PIN 8
#define DHT11_PIN 2 //can be any digital pin (might require PWM, not sure)
// #define LEFT 3   //Left most button (A)
// #define MIDDLE 4 //Middle Button (B)
// #define RIGHT 5 //Right most button (C)
#define FLOAT 6
#define TDS_PIN A1 // TDS pin, can be any analog
#define LIGHT_PIN A6 //can be any analog pin
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

#define Left        3 //Left most button (A)
#define Middle      4 //Middle Button (B)
#define Right       5 //Right most button (C)
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
//Target Variables
int tempTarget = 0;
int humidTarget = 0;
int pHTarget = 0;
int lightTarget = 0;
int tdsTarget = 0;
int adjustLights = 0;
int adjustPump = 0;
//Empty Variables for testing Pump and Adjust Lights screen
//Variables for clock compare
int yearsCompare = 0;
int daysCompare = 0;
int hoursCompare = 0;
int minutesCompare = 0;
int monthsCompare = 0;
//Straight Boolean
bool timeArrayPump[] = {false,false,false,false,false,false,false,true,true,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true,true,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true,true,false,false,false,false,false,false,false,false,false,false,false,false,true,true,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true,true,false,false,false,false,false,false,false,false,false,false,true,true,false,false,false,false,false,false,false,false,false,false,false,false,false};
bool timeArrayLights[] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,false,false,false,false,false,false,false,false,false,false,false,false,false};
//Water sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

  


void passiveBuzz()
  {
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
void readRTC( int *Hour, int *Minute){
  //This function reads the current time from the RTC and returns the values through the called pointers
  //for some reason, it needs these dummy variables to not throw a hissy fit
  bool garbageBool2 = false;
  bool garbageBool3 = false;
  *Hour = myRTC.getHour(garbageBool2,garbageBool3);
  *Minute = myRTC.getMinute();

  return;
}

void printRTC(){

  //This function calls the readRTC function and prints the returned values to the serial USB connection
  // int Year;
  // int Month;
  int Hour;
  int Minute;
  // int Second;
  readRTC(&Hour, &Minute);
  Serial.print("RTC Reading:\n");
  Serial.print("Time:");
  // Serial.print(" Y ");
  // Serial.print(Year);
  // Serial.print(" M ");
  // Serial.print(Month);
  Serial.print(" H ");
  Serial.print(Hour);
  Serial.print(" M ");
  Serial.print(Minute);
  // Serial.print(" S ");
  // Serial.print(Second);
  // Serial.print("\n");
  return;
}

void readDHT(double *Humidity, double *Temperature){

  //This function reads the current Temperature and Humidity values from the DHT sensor and returns those values through the called pointers (in percentage for Humidity and degrees C for Temperature)
  DHT.read11(DHT11_PIN);
  *Humidity = DHT.humidity;
 // *Temperature = DHT.temperature;
  *Temperature = (DHT.temperature * 1.80) + 32.00;
  return;
}
void readWaterTemp(float *WaterTemp)
{
  sensors.requestTemperatures();
  *WaterTemp = sensors.getTempFByIndex(0);
  return;
}
// void printDHT(){
//   //This function calls the readDHT function and prints the returned values to the serial USB connection
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

// void readTemp(double *TemperatureC, double *TemperatureF){
//   //This function reads the current Temperature Values (Celcius and Fahrenheit) and returns the values through the called pointers (in degrees)
//   double AbsoluteTemp = analogRead(TEMP_PIN);
//   //the formula below will need tweaking and calibration, but it's probably close enough for now
//   AbsoluteTemp = 124 - (AbsoluteTemp * 0.50);
//   *TemperatureC = AbsoluteTemp;
//   *TemperatureF = (AbsoluteTemp * 1.80) + 32.00;
//   return;
// }

// void printTemp(){
//   //This function calls the readTemp function and prints the returned values to the serial USB connection
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

void readLight(double *LightLevel){
  //This function reads the current light level from the Photoresistor and returns the value through the called pointers (as a percentage)
  double AbsoluteLight = analogRead(LIGHT_PIN);
  //just doing a percentage for now, we can decide on a unit (if we want one) later
  AbsoluteLight = AbsoluteLight / 1024.00;
  *LightLevel = 100.00 - (AbsoluteLight * 100.00);
  return;
}

void printLight(){
  //This function calls the readLight function and prints the returned value to the serial USB connection
  double LightLevel;
  readLight(&LightLevel);
  Serial.print("Light% Reading:\n");
  Serial.print("Light level: ");
  Serial.print(LightLevel);
  Serial.print("%\n");
  return;
}

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
    
  return;
}

void printTDS(){ //Doing Math for TDS


  readTDS();
  static unsigned long printTimepoint = millis();
 if(millis()-printTimepoint > 800U)
  {
    printTimepoint = millis();
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
    analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF/ 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient=1.0+0.02*(temperature-25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge=averageVoltage/compensationCoefficient; //temperature compensation
    tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue,0);
    Serial.println("ppm");
  }
  return;
}

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

void readPH(){
  measurings=0;
    for (int i = 0; i < samples; i++)
    {
        measurings += analogRead(pHSense);
        delay(10);

    }

    voltage = 5 / adc_resolution * measurings/samples;

    // for (int i = 0; i < samples; i++)
    // {
    //     measurings += analogRead(pHSense);
    //     delay(10);

    // }
  return;
}

void printPH(){
  readPH();
  Serial.print("pH= ");
  Serial.println((7 + ((2.5 - voltage) / 0.18)) + calibrate);
  // voltage = 5 / adc_resolution * measurings/samples;
  // Serial.print("pH= ");
  // Serial.println(ph(voltage));
  return;
}

void readWater(){
    water = digitalRead(FLOAT);   // read the input pin
  return;
}

// void printWater(){
//   readWater();
//   Serial.print("float= ");
//   Serial.println(water);
//   return;
// }

void setLightRelay(int state){
  //this function triggers the LED lights relay closed when true and open when false
  if(state == 1){
    digitalWrite(LIGHT_RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(LIGHT_RELAY_PIN, LOW);
  }
}

void setPumpRelay(int state){
  //this function triggers the pump relay closed when true and open when false
  if(state == 1){
    digitalWrite(PUMP_RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(PUMP_RELAY_PIN, LOW);
  }
}
void clockCompare()
{
  readRTC(hoursCompare, minutesCompare );
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
bool startPumpTime(int *settingTime)
{
   while(digitalRead(Middle)==HIGH)
   {
    int displayHours = *settingTime/4;
    int displayMinute = (*settingTime%4)*15;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Start of Pump ON");
    lcd.setCursor(0,1);
    lcd.print("Time:");
    if(displayHours > 9)
    {
      lcd.setCursor(0,2);
    }
    else
    {
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
           *settingTime++;
              
         }
      if(*settingTime < 0)
      {
        *settingTime = 96;
      }
      if(*settingTime > 96)
      {
        bool leaveMeAlone = false;
         while(leaveMeAlone == false)
         {
           lcd.clear();
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
               delay(100);
              leaveMeAlone = true;
              *settingTime = 96;              
             }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
             {
                //  pressedButton = true;
                delay(100);
                return true;         
             }
           }
          delay(100);
      }

   }
   
   return false;
}
void endPumpTime(int *settingTime)
{
   int startSettingTime = *settingTime;
   *settingTime++;
   while(digitalRead(Middle)==HIGH)
   {
    int displayHours = *settingTime/4;
    int displayMinute = (*settingTime%4)*15;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("End of Pump ON");
    lcd.setCursor(0,1);
    lcd.print("Time:");
    if(displayHours > 9)
    {
      lcd.setCursor(0,2);
    }
    else
    {
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
           *settingTime++;
              
         }
      if(*settingTime < 0)
      {
        *settingTime = 96;
      }
      if(*settingTime > 96)
      {
        *settingTime = 0;
      }

   }
   for(int i = startSettingTime; i < *settingTime;i++)
   {
     timeArrayPump[i] = true;
   }
   return;
}
void pumpAdjustArray()
  {
        bool leaveMeAlone = false;
         while(leaveMeAlone == false)
         {
           lcd.clear();
           lcd.setCursor(0,0);
           lcd.print("Do you want to");
           lcd.setCursor(0,1);
           lcd.print("Adjust Pump Times?");
           lcd.setCursor(0,2);
           lcd.print("NO               YES");
           lcd.setCursor(0,3);
           lcd.print("<--              -->");
           lcd.display();

           if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
             {
            //  pressedButton = true;
               delay(100);
               screenNumber = 14;
              leaveMeAlone = true;
              return;           
             }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
             {
                //  pressedButton = true;
                delay(100);
                leaveMeAlone = true;         
             }
           }
    for(int i = 0; i<96; i++)
    {
      timeArrayPump[i] = false;
    }
    int settingTime = 0;

    bool exitcondition = false;
    while(exitcondition == false)
    {
        exitcondition = startPumpTime(&settingTime);
        if(exitcondition == false)
        {
          endPumpTime(&settingTime);
        }
    }
    return; 
  }
bool startLightTime(int *settingTime)
{
   while(digitalRead(Middle)==HIGH)
   {
    int displayHours = *settingTime/4;
    int displayMinute = (*settingTime%4)*15;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Start of Light ON");
    lcd.setCursor(0,1);
    lcd.print("Time:");
    if(displayHours > 9)
    {
      lcd.setCursor(0,2);
    }
    else
    {
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
           *settingTime++;
              
         }
      if(*settingTime < 0)
      {
        *settingTime = 96;
      }
      if(*settingTime > 96)
      {
        bool leaveMeAlone = false;
         while(leaveMeAlone == false)
         {
           lcd.clear();
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
               delay(100);
              leaveMeAlone = true;
              *settingTime = 96;              
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

   }
   
   return false;
}
void endLightTime(int *settingTime)
{
   int startSettingTime = *settingTime;
   *settingTime++;
   while(digitalRead(Middle)==HIGH)
   {
    int displayHours = *settingTime/4;
    int displayMinute = (*settingTime%4)*15;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("End of Light ON");
    lcd.setCursor(0,1);
    lcd.print("Time:");
    if(displayHours > 9)
    {
      lcd.setCursor(0,2);
    }
    else
    {
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
           *settingTime++;
              
         }
      if(*settingTime < 0)
      {
        *settingTime = 96;
      }
      if(*settingTime > 96)
      {
        *settingTime = 0;
      }

   }
   for(int i = startSettingTime; i < *settingTime;i++)
   {
     timeArrayLights[i] = true;
   }
   return;
}
void lightAdjustArray()
  {
          bool leaveMeAlone = false;
         while(leaveMeAlone == false)
         {
           lcd.clear();
           lcd.setCursor(0,0);
           lcd.print("Do you want to");
           lcd.setCursor(0,1);
           lcd.print("Adjust Light Times?");
           lcd.setCursor(0,2);
           lcd.print("NO               YES");
           lcd.setCursor(0,3);
           lcd.print("<--              -->");
           lcd.display();

           if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
             {
            //  pressedButton = true;
               delay(100);
               screenNumber = 12;
              leaveMeAlone = true;
              return;           
             }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
             {
                //  pressedButton = true;
                delay(100);
                leaveMeAlone = true;         
             }
           }
    for(int i = 0; i<96; i++)
    {
      timeArrayLights[i] = false;
    }
    int settingTime = 0;

    bool exitcondition = false;
    while(exitcondition == false)
    {
        exitcondition = startLightTime(&settingTime);
        if(exitcondition == false)
        {
          endLightTime(&settingTime);
        }
    }
    return; 
  }
void maintDisplay(int sensorValue, String units, String sensorName, String lastNextScreen){
    lcd.clear();
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

void screenDisplay(int sensorValue, String units, String sensorName, String lastNextScreen){ //Function for formatting screens
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(sensorName);
    lcd.setCursor(7,1);
    lcd.print(sensorValue);
    lcd.setCursor(11,1);
    lcd.print(units);   
    lcd.setCursor(0, 3);
    lcd.print(lastNextScreen);  
    lcd.display();
    return;

}
void updateValues()
  {
   readDHT(&temp,&humid);
   readWaterTemp(&waterTemp);  
   readWater();
   readLight(&lightLvl);
   readTDS();
   readPH();
  }
//void statement to check if variable is within spread
bool spreadChecker ()
{
  bool good = true;
  if(abs(temp-tempTarget) > TemperatureSpread)
  {
    good = false;
    screenNumber = 2;
    lcd.setCursor(0,2);
    lcd.print("Temp out of range!");
  }
  if(abs(humid-humidTarget) > HumiditySpread)
  {
    good = false;
    screenNumber = 3;
    lcd.setCursor(0,2);
    lcd.print("Humid out of range!");
  }
  if(abs(tdsValue-tdsTarget) > TDSSpread)
  {
    good = false;
    screenNumber = 4;
    lcd.setCursor(0,2);
    lcd.print("TDS out of range!");
  }
  if(abs(lightLvl-lightTarget) > LightLevelSpread)
  {
    good = false;
    screenNumber = 7;
    lcd.setCursor(0,2);
    lcd.print("Light out of range!");
  }
  if(abs(pHLvl-pHTarget) > pHSpread)
  {
    good = false;
    screenNumber = 6;
    lcd.setCursor(0,2);
    lcd.print("pHLvl out of range!");
  }
  if(water == 0)
  {
    good = false;
    screenNumber = 5;
    lcd.setCursor(0,2);
    lcd.print("No Water Detected!");
  }  
  
  return good;
}
void updateScreen(){
 
  if(pressedButton == true)
  { //Checking if button has been pressed
 
    switch(screenNumber)
    {
      case 1:
        lcd.clear();
        lcd.setCursor(6,1);
        lcd.print("Monitor");
        lcd.setCursor(6,2);
        lcd.print("Screens");
        lcd.display();
        maintNumber = 0;
      break;
      
      case 2: //Temp sensor
        readDHT(&temp,&humid);
        readWaterTemp(&waterTemp);
        screenDisplay(temp, "F", "Temperature:", "<-MonitorSCR Humid->"); //Calling screenDisplay and making screen for variable
        lcd.setCursor(0,2);
        lcd.print("Water Temp:"); 
        lcd.setCursor(12,2);
        lcd.print(waterTemp);
        lcd.setCursor(15,2);   
        lcd.print("F");                   
        maintNumber = 0;
      break;
  
      case 3: //Humidity Sensor
        readDHT(&temp,&humid);
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
        screenDisplay(water, "    0=No", "Water Present? 1=Yes", "<- TDS         pH ->");
        maintNumber = 0;
      break;

      case 6: //pH sensor
       readPH();
       screenDisplay(pHLvl, "pH", "pH:", "<-WaterLVL  Light%->");
       maintNumber = 0;
      break;

      case 7:// Light% 
        readLight(&lightLvl);
        screenDisplay(lightLvl, "%", "Light Level:", "<- pH           SD->");
        maintNumber = 0;
      break;

      // case 8: //SD  Card
      //   screenDisplay(tdsValue, "    TDS=    ppm", "WaterLVL->", "<- Humid");  
      //   maintNumber = 0;
      // break;

      case 8:
        lcd.clear();
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
        lcd.clear();
        lcd.setCursor(4,1);
        lcd.print("Maintenance");
        lcd.setCursor(6,2);
        lcd.print("Screens");
        lcd.display();
        maintNumber = 0;
      break;

      case 10: //Adjust Light Screen
        maintDisplay(adjustLights, "    0=OFF", "Lights:        1=ON", "<- MaintSC ADJPump->");
        maintNumber = 1;    
      break;
      case 11: //Adjust Light Array Screen
        lightAdjustArray();
      break;

      case 12: //Adjust Pump Screen
        maintDisplay(adjustPump, "    0=OFF", "Pump:          1=ON", "<-ADJLGT TempSens->"); //Calling screenDisplay and making screen for variable
        maintNumber = 2; 
      break;
      case 13: //Adjust Pump Array Screen
      pumpAdjustArray();
      break;
  
      case 14: //Temp sensor maintenance
        maintDisplay(tempTarget, "F", "Target Temperature:", "<-ADJPump    Humid->");
        maintNumber = 3; 
      break;

      case 15: //Humidity Sensor maintenance
        maintDisplay(humidTarget, "%", "Target Humidity:", "<-TempSens TDSSens->");
        maintNumber = 4; 
      break;

      case 16:  //TDS sensor maintenance
        maintDisplay(tdsTarget, "ppm", "Target TDS:", "<-Humid   WaterLVL->");
        maintNumber = 5; 
      break;

      case 17: //pH sensor maintenance
        maintDisplay(pHTarget, "pH", "Target pH:", "<-WaterLVL Light %->");
        maintNumber = 6; 
      break;

      case 18:// Light%  maintenance
        maintDisplay(lightLvl, "%", "Target LightLVL:", "<-pH     MaintSCRN->");
        maintNumber = 7; 
      break;

      case 19:// Light%  maintenance
          lcd.clear();
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

  delay(1000);
  clockCompare();
  setPumpRelay(adjustPump);
  setLightRelay(adjustLights);
  updateValues();
  if(spreadChecker() == false)
  {
    error++;
    if(error >= 10)
    {

    while(digitalRead(Middle)==HIGH)
    {
      passiveBuzz(); //Alarm Functions if values are not in spread
      delay(1000);
    }
    error = 0;
    }
  }
  pressedButton = false;
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
              maintDisplay(tempTarget, "F", "Target Temperature:", "<-ADJPump    Humid->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              tempTarget++;
              maintDisplay(tempTarget, "F", "Target Temperature:", "<-ADJPump    Humid->");
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
              humidTarget = humidTarget - 1;
              maintDisplay(humidTarget, "%", "Target Humidity:", "<-TempSens TDSSens->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              humidTarget++;
              maintDisplay(humidTarget, "%", "Target Humidity:", "<-TempSens TDSSens->");
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
              maintDisplay(tdsTarget, "ppm", "Target TDS:", "<-Humid WaterLVL->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              tdsTarget++;
              maintDisplay(tdsTarget, "ppm", "Target TDS:", "<-Humid WaterLVL->");
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
        case 7:
          if(digitalRead(Left)==LOW) //Check for if Left button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              lightTarget = lightTarget - 1;
              maintDisplay(lightTarget, "%", "Target LightLVL:", "<-pH     MaintSCRN->"); 
             // Serial.println("I have decreased the light level by 1"); Test to ensure you are adjusting values
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              lightTarget++;
              maintDisplay(lightTarget, "%", "Target LightLVL:", "<-pH     MaintSCRN->"); 
              // Serial.println("I have increased the light level by 1");      Test to ensure you are adjusting values       
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



void setup() {
  Wire.begin();
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(TDS_PIN,INPUT);
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

void loop() {
  // Serial.print("\n\n");
  // Serial.print("---------------------------\n");
  // printRTC();
  // printDHT();
 // printLight();
  // printTDS();
  // printPH();
  // printWater();
  updateScreen();


}