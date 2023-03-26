//this is a test pt2

#include <time.h>
#include <Wire.h>
//library for RTC
#include <DS3231.h>
//library for DHT sensor
#include <dht.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

DS3231 myRTC;
dht DHT;
LiquidCrystal_I2C lcd(0x27,20,4);

int pHSense = A0;
int samples = 10;
float calibrate = 2.12;
float adc_resolution = 1024.0;
#define PASSIVE_BUZZER_PIN 11
#define LIGHT_RELAY_PIN 7
#define PUMP_RELAY_PIN 8
#define DHT11_PIN 2 //can be any digital pin (might require PWM, not sure)
// #define LEFT 3
// #define MIDDLE 4
// #define RIGHT 5
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

#define Left        2 //Left most button (A)
#define Middle      3 //Middle Button (B)
#define Right       4 //Right most button (C)
//Global variables
int screenNumber = 1;
int maintNumber = 0;
bool pressedButton = true;
bool middleButton = false;
bool middleButton2 = false;
//Global Variables for Sensors
int tempValueF = 30;
int humidValue = 30;
//int tdsValue = 300;
int waterLvl = 1;
int pHLvl = 10;
int lightLvl = 50;
//Empty Variables for testing Pump and Adjust Lights screen
int adjustPump = 1;
int adjustLights = 1;
  

    
void readRTC(int *Year, int *Month, int *Date, int *Hour, int *Minute, int *Second){
  //This function reads the current time from the RTC and returns the values through the called pointers
  //for some reason, it needs these dummy variables to not throw a hissy fit
  bool garbageBool = false;
  bool garbageBool2 = false;
  bool garbageBool3 = false;
  *Year = myRTC.getYear();
  *Month = myRTC.getMonth(garbageBool);
  *Date = myRTC.getDate();
  *Hour = myRTC.getHour(garbageBool2,garbageBool3);
  *Minute = myRTC.getMinute();
  *Second = myRTC.getSecond();
  return;
}

void printRTC(){

  //This function calls the readRTC function and prints the returned values to the serial USB connection
  int Year;
  int Month;
  int Date;
  int Hour;
  int Minute;
  int Second;
  readRTC(&Year, &Month, &Date, &Hour, &Minute, &Second);
  Serial.print("RTC Reading:\n");
  Serial.print("Time:");
  Serial.print(" Y ");
  Serial.print(Year);
  Serial.print(" M ");
  Serial.print(Month);
  Serial.print(" D ");
  Serial.print(Date);
  Serial.print(" H ");
  Serial.print(Hour);
  Serial.print(" M ");
  Serial.print(Minute);
  Serial.print(" S ");
  Serial.print(Second);
  Serial.print("\n");
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

void printDHT(){
  //This function calls the readDHT function and prints the returned values to the serial USB connection
  double Humidity;
  double Temperature;
  readDHT(&Humidity, &Temperature);
  Serial.print("DHT Reading:  ");
  Serial.print("Humidity: ");
  Serial.print(Humidity);
  Serial.print(" Temperature: ");
  Serial.print(Temperature);
  Serial.print("\n");
  return;
}

void readTemp(double *TemperatureC, double *TemperatureF){
  //This function reads the current Temperature Values (Celcius and Fahrenheit) and returns the values through the called pointers (in degrees)
  double AbsoluteTemp = analogRead(TEMP_PIN);
  //the formula below will need tweaking and calibration, but it's probably close enough for now
  AbsoluteTemp = 124 - (AbsoluteTemp * 0.50);
  *TemperatureC = AbsoluteTemp;
  *TemperatureF = (AbsoluteTemp * 1.80) + 32.00;
  return;
}

void printTemp(){
  //This function calls the readTemp function and prints the returned values to the serial USB connection
  double TemperatureC;
  double TemperatureF;
  readTemp(&TemperatureC, &TemperatureF);
  Serial.print("Temperature Reading:\n");
  Serial.print("Fahrenheit: ");
  Serial.print(TemperatureF);
  Serial.print(" Celcius: ");
  Serial.print(TemperatureC);
  Serial.print("\n");
  return;
}

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

void printTDS(){


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

void printWater(){
  readWater();
  Serial.print("float= ");
  Serial.println(water);
  return;
}

void setLightRelay(bool state){
  //this function triggers the LED lights relay closed when true and open when false
  if(state){
    digitalWrite(LIGHT_RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(LIGHT_RELAY_PIN, LOW);
  }
}

void setPumpRelay(bool state){
  //this function triggers the pump relay closed when true and open when false
  if(state){
    digitalWrite(PUMP_RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(PUMP_RELAY_PIN, LOW);
  }
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
    //continue from here
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
        screenDisplay(tempValueF, "F", "Temperature:", "<-MonitorSCR Humid->"); //Calling screenDisplay and making screen for variable
        maintNumber = 0;
      break;
  
      case 3: //Humidity Sensor
        screenDisplay(humidValue, "%", "Humidity:", "<- Temp       TDS ->");
        maintNumber = 0;
      break;

      case 4: //TDS sensor
        screenDisplay(tdsValue, "ppm", "TDS:", "<- Humid  WaterLVL->");
        maintNumber = 0;
      break;

      case 5: //Water Level Sensor        
        screenDisplay(waterLvl, "    0=No", "Water Present? 1=Yes", "<- TDS         pH ->");
        maintNumber = 0;
      break;

      case 6: //pH sensor
       screenDisplay(pHLvl, "pH", "pH:", "<-WaterLVL  Light%->");
       maintNumber = 0;
      break;

      case 7:// Light% 
        screenDisplay(lightLvl, "%", "Light Level:", "<- pH           SD->");
        maintNumber = 0;
      break;

      case 8: //SD  Card
        screenDisplay(tdsValue, "    TDS=    ppm", "WaterLVL->", "<- Humid");  
        maintNumber = 0;
      break;

      case 9:
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
      case 10:
        lcd.clear();
        lcd.setCursor(4,1);
        lcd.print("Maintenance");
        lcd.setCursor(6,2);
        lcd.print("Screens");
        lcd.display();
        maintNumber = 0;
      break;

      case 11: //Adjust Light Screen
        maintDisplay(adjustLights, "    0=OFF", "Lights:        1=ON", "<- MaintSC ADJPump->");
        maintNumber = 1;    
      break;

      case 12: //Adjust Pump Screen
        maintDisplay(adjustPump, "    0=OFF", "Pump:          1=ON", "<-ADJLGT TempSens->"); //Calling screenDisplay and making screen for variable
        maintNumber = 2; 
      break;
  
      case 13: //Temp sensor maintenance
        maintDisplay(tempValueF, "F", "Target Temperature:", "<-ADJPump    Humid->");
        maintNumber = 3; 
      break;

      case 14: //Humidity Sensor maintenance
        maintDisplay(humidValue, "%", "Target Humidity:", "<-TempSens TDSSens->");
        maintNumber = 4; 
      break;

      case 15:  //TDS sensor maintenance
        maintDisplay(tdsValue, "ppm", "Target TDS:", "<-Humid   WaterLVL->");
        maintNumber = 5; 
      break;

      case 16: //pH sensor maintenance
        maintDisplay(pHLvl, "pH", "Target pH:", "<-WaterLVL Light %->");
        maintNumber = 6; 
      break;

      case 17:// Light%  maintenance
        maintDisplay(lightLvl, "%", "Target LightLVL:", "<-pH     MaintSCRN->");
        maintNumber = 7; 
      break;

      case 18:// Light%  maintenance
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
              tempValueF = tempValueF - 1;
              maintDisplay(tempValueF, "F", "Target Temperature:", "<-ADJPump    Humid->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              tempValueF++;
              maintDisplay(tempValueF, "F", "Target Temperature:", "<-ADJPump    Humid->");
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
              humidValue = humidValue - 1;
              maintDisplay(humidValue, "%", "Target Humidity:", "<-TempSens TDSSens->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              humidValue++;
              maintDisplay(humidValue, "%", "Target Humidity:", "<-TempSens TDSSens->");
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
              tdsValue = tdsValue - 1;
              maintDisplay(tdsValue, "ppm", "Target TDS:", "<-Humid WaterLVL->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              tdsValue++;
              maintDisplay(tdsValue, "ppm", "Target TDS:", "<-Humid WaterLVL->");
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
              pHLvl = pHLvl - 1;
              maintDisplay(pHLvl, "pH", "Target pH:", "<-WaterLVL Light %->");
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
             // pressedButton = true;
              delay(100);
              pHLvl++;
              maintDisplay(pHLvl, "pH", "Target pH:", "<-WaterLVL Light %->");
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
              lightLvl = lightLvl - 1;
              maintDisplay(lightLvl, "%", "Target LightLVL:", "<-pH     MaintSCRN->"); 
             // Serial.println("I have decreased the light level by 1"); Test to ensure you are adjusting values
            }
          else if(digitalRead(Right)==LOW) //Check for if Right button pressed then move case number 
            {
            //  pressedButton = true;
              delay(100);
              lightLvl++;
              maintDisplay(lightLvl, "%", "Target LightLVL:", "<-pH     MaintSCRN->"); 
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
  // pinMode(LEFT, INPUT);
  // pinMode(MIDDLE, INPUT);
  // pinMode(RIGHT, INPUT);
  pinMode(FLOAT, INPUT);

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
  Serial.print("\n\n");
  Serial.print("---------------------------\n");
  printRTC();
  printDHT();
 // printLight();
  printTDS();
  printPH();
  printWater();
  updateScreen();


}