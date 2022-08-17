//Code V2.1.4
//this could be used for AirSENSE version 2 and version 2.1
//in this version, i used 2 sensors PMS7003 and DHT22
//the RTC module DS3231, SD card and LCD using I2C

//1. include library

#include <SoftwareSerial.h>         
#include <SPI.h>
#include <SD.h>                     //SD card
#include <dht_nonblocking.h>       //DHT22
#include <DS3231.h>                 //RTC module
#include <Wire.h>                 
#include <LiquidCrystal_I2C.h>       //LCD
#include <PMS.h>                    //PMS7003

//2. define 
#define DHT_SENSOR_TYPE DHT_TYPE_22

#define TX_PIN_DUST 4
#define RX_PIN_DUST 5   
////
//#define DHT_SENSOR_PIN  A3 //for v2.1
////#define PIN_BUTTON    8 //for v2.1
//#define PIN_CS_SD 7     //for v2.1

#define PIN_CS_SD 10    // for v2.0
#define DHT_SENSOR_PIN  7 // for v2.0
#define PIN_BUTTON    2 // for v2.0

#define PACKET_ESP_SIZE 8 //data from ESP to arduino
#define PACKET_TO_ESP_SIZE 24 //data from arduino to ESP

// globle variable 
float Temperature;
float Humidity;
float temperatureSum;
float humiditySum;
float PM1;
float PM1Sum;
float PM10;
float PM10Sum;
float PM2_5;
float PM2_5Sum;
float CO; //CO concentration at the end of previous measurement cycle


uint8_t displayFlag = 0;// display flag, 0 means display pm2.5 hum and tem, 1 means display co and time, 2 means turn off lcd
uint8_t controlFlag = 0;  //complicated  flags
//there are 8 flag bits for controlling the sequence, they are set when the RTC alarms every 30 seconds
//actually, there are two alarm: alarm1 set every 30th second in a minute(00:30 01:30 02:30 ...), alarm2 set every 00 second in a minute (00:00 01:00 02:00 ...)
//when alarm1 set, 

//Count variable 
uint8_t countDHTData = 0;
uint8_t countPMSData = 0;
uint8_t countDataByte = 0;

//data that recivied from ESP it an array of 8 including 2 ID bytes, 4 bytes of unixtime and 2 by fcs bytes
uint8_t dataBuffer[PACKET_ESP_SIZE];

unsigned long lastReadDHT = 0;
unsigned long lastDisplay = 0;

//initialize sensor and module once
SoftwareSerial SerialPMS = SoftwareSerial(TX_PIN_DUST, RX_PIN_DUST);
PMS pms(SerialPMS);
PMS::DATA data;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
LiquidCrystal_I2C lcd(0x27, 16, 2);
DS3231 Clock;  
RTCDateTime dt; // this is the time structure including unixtime

//3. functions were used 
//display function
//because we have 3 display modes: pm2.5 tem and hum, time and CO, and stop display
void lcdInit();
void displayPM2p5();
void displayTimeAndCO();
void displayDayOfWeek(uint8_t day);
void displayTemAndHum();
void displayNumber();
void stopDisplay();
void displayAgain();
void setDisplayScreen(uint8_t state);//depend on the button to display in LCD
bool button(uint8_t pin);         //check if button is pressed
//these functions below are for reading and processing data
void checkAlarm();  // alarm for every 30 seconds to set ControlFag bits
void getPMSData();
void getDHTData();
void getMQ7Data(); // this fuction is about faking data :), you should put on it right code
void process_logDataSD_sendData2ESP();//processing data then log it to SD card and send to ESP

void setup() 
{
  lcdInit();
  Serial.begin(9600);
  SerialPMS.begin(9600);
  pms.passiveMode();    // Switch to passive mode
  
  pinMode(PIN_BUTTON  , INPUT);
  
  Wire.begin();
  Clock.begin();
  // Disarm alarms and clear alarms for this example, because alarms is battery backed.
  // Under normal conditions, the settings should be reset after power and restart microcontroller.
  Clock.armAlarm1(false);
  Clock.armAlarm2(false);
  Clock.clearAlarm1();
  Clock.clearAlarm2();

  // Set Alarm - Every full minute.
  // DS3231_EVERY_MINUTE is available only on Alarm2.
  // setAlarm2(Date or Day, Hour, Minute, Mode, Armed = true)
  Clock.setAlarm2(0, 0, 0, DS3231_EVERY_MINUTE,true);

  // Set Alarm1 - Every 30s in each minute
  // setAlarm1(Date or Day, Hour, Minute, Second, Mode, Armed = true)
  Clock.setAlarm1(0, 0, 0, 30, DS3231_MATCH_S,true);

  SD.begin(PIN_CS_SD);
  
  lcd.clear();
}

//4. Now the main loop

void loop () 
{
  checkAlarm();  
  getPMSData();
  getDHTData();
  getMQ7Data();
  
  if(button(PIN_BUTTON))
  {
    displayFlag++;
    if(displayFlag > 2)
      displayFlag = 0;
    lcd.clear();
    lastDisplay = millis() + 1000;
  } 
  if (Serial.available() > 0)
  {
    //read dust, mes, time
//    uint8_t dataBuffer[PACKET_ESP_SIZE];
    dataBuffer[countDataByte] = Serial.read();
    if (dataBuffer[0] == 66)
      countDataByte++;
    if (countDataByte == PACKET_ESP_SIZE)
    {
      countDataByte = 0;
      uint16_t check = 0;
      for (uint8_t i = 0; i < PACKET_ESP_SIZE - 2; i++)
      {
        check += dataBuffer[i];
      }
      uint16_t check2 = (dataBuffer[PACKET_ESP_SIZE - 2] << 8) + dataBuffer[PACKET_ESP_SIZE - 1];
      if (check == check2)
      {
        if (dataBuffer[1] == 88)
        {
          uint32_t internetTime = ((uint32_t)dataBuffer[2] << 24) + ((uint32_t)dataBuffer[3] << 16) + ((uint32_t)dataBuffer[4] << 8) + dataBuffer[5];
          dt = Clock.getDateTime();
          if (abs(internetTime - dt.unixtime) > 3)
          {
            Clock.setDateTime(internetTime);
          } 
        }
      }
    }
  }

  else if((controlFlag & 0b00000011) == 3)//check 1st and 2nd bit
  {
    dt = Clock.getDateTime();
    process_logDataSD_sendData2ESP();
    controlFlag &= 0b11111100;
  } 
  else if((lastDisplay == 0) || ((millis() - lastDisplay) >999))
  {
      lastDisplay = millis();
      setDisplayScreen(displayFlag);
  }
}

void checkAlarm()
{
  if(Clock.isAlarm1())
  { 
    controlFlag |= 0b01010100;//set fag sleep PMS,
  }
  if(Clock.isAlarm2())
  {
    controlFlag |= 0b10101000;//set bit wake PMS, 
    controlFlag ++; // increase value to log data
  }
}
void getDHTData()
{
  if ( millis( ) - lastReadDHT > 2500 )
  {
    if ( dht_sensor.measure( &Temperature, &Humidity) == true )
    {
      lastReadDHT = millis( );        
      humiditySum += Humidity;
      temperatureSum += Temperature;
      countDHTData++;
      if(displayFlag == 0)
      {
        lcd.setCursor(2, 1); //Colum-Row
        lcd.print(Humidity, 1);
        lcd.setCursor(10, 1);
        lcd.print(Temperature, 1);
      }     
    }   
  }
}
void getPMSData()
{
  // wake it up and wait for 30 seconds
  // then get the data
  // after that leep PMS and wait for 30 seconds
  // back to step one
  
  //pms.wakeUp();
  //if(countState = 0)
  //if(Clock.isAlarm1())
  if(controlFlag & 0b00001000)//check 4th flag
  {
    pms.wakeUp();       //wake it up
    controlFlag &= 0b11110111; //turn off flag
  }
  //else if(CountState = 1)
  //if(Clock.isAlarm2())
  if(controlFlag & 0b00000100)//check 3rd flag
  {
    pms.requestRead();
    if(pms.readUntil(data))
    {
      PM1   = data.PM_AE_UG_1_0;
      PM2_5 = data.PM_AE_UG_2_5;
      PM10  = data.PM_AE_UG_10_0;
      PM1Sum   += PM1;
      PM2_5Sum += PM2_5;
      PM10Sum  += PM10;
      countPMSData++;
      if(displayFlag == 0)
      {
        lcd.setCursor(7, 0);
        lcd.print("    ");
        lcd.setCursor(7, 0);
        lcd.print((uint8_t)PM2_5);
      }
    }
    pms.sleep();
    controlFlag &= 0b11111011;
  }
}
void getMQ7Data()
{
  if(controlFlag & 0b01000000)
  {
    CO = random(7,12)+random(0,100)*0.01; // fake Co
    controlFlag &= 0b10111111;
    if(displayFlag == 1)
    {
      lcd.setCursor(6,1);
      lcd.print("     ");
      lcd.setCursor(6,1);
      lcd.print(CO,2);
    }
  }
}

void lcdInit()
{
    byte dot[8]={
    B00110,
    B01001,
    B01001,
    B00110,
    B00000,
    B00000,
    B00000,
    B00000
  };
  
  byte so3[8]={
    B11110,
    B00100,
    B01100,
    B00010,
    B10010,
    B01100,
    B00000,
    B00000
  };
  
  byte micro[8]={
    B00000,
    B00000,
    B10001,
    B10001,
    B10001,
    B10011,
    B01101,
    B10000
  };
  lcd.begin();
  lcd.createChar(0,dot);
  lcd.createChar(1,so3);
  lcd.createChar(2,micro);
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(" AirSENSE v2.1.4");
  lcd.setCursor(0,1);
  lcd.print(" by SPARClab    ");
  delay(500);
}

void displayTimeAndCO()
{
  lcd.setCursor(0,0);
  displayDayOfWeek(dt.dayOfWeek);
  //displayNumber(dt.dayOfWeek);
  lcd.print(":  ");
    
  lcd.setCursor(6,0);
  displayNumber(dt.hour);
  lcd.setCursor(8,0);
  lcd.print(":");
  lcd.setCursor(9,0);
  displayNumber(dt.minute);
    
  lcd.setCursor(11,0);
  lcd.print(":");
  lcd.setCursor(12,0);
  displayNumber(dt.second);
  lcd.print("  ");

  lcd.setCursor(0,1);
  lcd.print("CO :  ");
  if(CO == 0){
    lcd.print("----");
  }
  else {
    lcd.print(CO,2);
  }
  lcd.setCursor(11,1);
  lcd.print("pmm  ");
}
void displayPM2p5()
{
  lcd.setCursor(0, 0); //Colum-Row
  lcd.print("PM2.5: ");
  lcd.setCursor(7, 0);
  if(!PM2_5)
  {
    lcd.print("----");
  }
  else lcd.print((uint8_t)PM2_5);
  lcd.setCursor(11,0);
  //lcd.print(" ");
  lcd.write(byte(2));
  lcd.print("g/m");
  lcd.write(byte(1));
  //lcd.setCursor(6, 0);
}
void displayTemAndHum()
{  
  lcd.setCursor(0, 1); //Colum-Row
  lcd.print("H:");
  if(Humidity == 0){
      lcd.print("----");
  }
  else {
    lcd.print(Humidity, 1);
  }
    
  lcd.setCursor(6, 1);
  lcd.print("%");
  
  lcd.setCursor(8, 1); //Colum-Row
  lcd.print("T:");
  if(Temperature == 0){
    lcd.print("----");
  }
  else {
    lcd.print(Temperature, 1);
  }
  lcd.setCursor(14, 1);
  lcd.write(byte(0));
  lcd.print("C");
}

void displayDayOfWeek(uint8_t day)
{
  switch(day)
  {
    case 0: lcd.print("Sun");break;
    case 1: lcd.print("Mon");break;
    case 2: lcd.print("Tue");break;
    case 3: lcd.print("Wed");break;
    case 4: lcd.print("Thu");break;
    case 5: lcd.print("Fri");break;
    case 6: lcd.print("Sat");break;
    default: return 0;
  } 
}
void displayNumber(int num)
{
  lcd.print(num/10);
  lcd.print(num%10);
}

void stopDisplay()
{
  lcd.noDisplay();
  lcd.noBacklight();
}
void displayAgain()
{
  lcd.display();
  lcd.backlight();
}
bool button(uint8_t pin)
{
  if(!digitalRead(pin))
  {
    delay(1);
    if(!digitalRead(pin))
    {
      while(!digitalRead(pin));
      return (true);
    }
  }
  return (false);
}
void setDisplayScreen(uint8_t state)
{
  switch(state)
  {
    case 0:
      displayAgain();
      displayPM2p5();
      displayTemAndHum();
      break;
    case 1:
      dt = Clock.getDateTime();
      displayTimeAndCO();
      break;
    case 2:
      stopDisplay();
      break;
    default: break;
  }
}
void process_logDataSD_sendData2ESP()
{
  // average value
  float temp = 0;
  float hum = 0;  
  
  float pm1 = 0;
  float pm25 = 0;
  float pm10 = 0;
  
  float co = 0;
  
  if (countDHTData != 0)
  {
    temp = (float)temperatureSum / countDHTData;
    hum = (float)humiditySum / countDHTData;
    temperatureSum = 0;
    humiditySum = 0;
    countDHTData = 0;
  }

  if (countPMSData != 0)
  {
    pm1 = (float)PM1Sum / countPMSData;
    pm25 = (float)PM2_5Sum / countPMSData;
    pm10 = (float)PM10Sum / countPMSData;
    PM1Sum = 0;
    PM2_5Sum = 0;
    PM10Sum = 0;
    countPMSData = 0;
  }
  //if(PM2_5*PM10*PM1)
  /*{
    pm1 = PM1;
    pm25 = PM2_5;
    pm10 = PM10;
  }*/
  co = CO;
  /*if (dataMQ7Count != 0)
  {
    co = (float)COppmSum / dataMQ7Count;
    COppmSum = 0;
    dataMQ7Count = 0;
  }*/

  // cong thuc cua airbeam
  //        pm1=0.66776*pow(pm1,1.1);
  //        pm25=1.33*pow(pm25,0.85);
  //        pm10=1.06*pm10;
  
  //log data to SD 
  SD.begin(PIN_CS_SD);
  char fileName[12];
  sprintf(fileName, "%d-%d-%d.txt", dt.day, dt.month, dt.year - 2000);
  File f = SD.open(fileName, FILE_WRITE);

  if (f)
  {
    f.print(dt.year);
    f.print("-");
    f.print(dt.month);
    f.print("-");
    f.print(dt.day);
    f.print(" ");

    f.print(dt.hour);
    f.print(":");
    f.print(dt.minute);
    f.print(":");
    f.print(dt.second);
    f.print(",");
        
    f.print(dt.unixtime);
    f.print(",");
    
    f.print(temp);
    f.print(",");
    f.print(hum);
    f.print(",");

    f.print(pm1);
    f.print(",");
    f.print(pm25);
    f.print(",");
    f.print(pm10);
    f.print(",");

    f.println(co);
   
    f.close();
  }
  // end logging data to SD card
  
  //prepare data to send to ESP
  //because temperature, humidity, particles matter and CO are float value
  //I devided them into 2 parts the interger part and decemal part
  //interger part
  uint8_t _temp = temp;
  uint8_t _hum = hum;
  uint16_t _pm1 = pm1;
  uint16_t _pm25 = pm25;
  uint16_t _pm10 = pm10;
  uint16_t _co = co;

  //decemal part
  uint8_t dot_temp = (temp - _temp) * 100;
  uint8_t dot_hum = (hum - _hum) * 100;
  uint8_t dot_pm1 = (pm1 - _pm1) * 100;
  uint8_t dot_pm25 = (pm25 - _pm25) * 100;
  uint8_t dot_pm10 = (pm10 - _pm10) * 100;
  uint8_t dot_co = (co - _co) * 100;

  uint32_t arduinoTime = dt.unixtime;

  uint8_t dataToEspBuffer[PACKET_TO_ESP_SIZE] = {66, 77, _temp, dot_temp, _hum, dot_hum, (_pm1 >> 8), (_pm1 & 0xff), dot_pm1, (_pm25 >> 8), (_pm25 & 0xff), dot_pm25, (_pm10 >> 8), (_pm10 & 0xff), dot_pm10, (_co >> 8), (_co & 0xff), dot_co, 0, 0, 0, 0, 0, 0};

  dataToEspBuffer[PACKET_TO_ESP_SIZE - 3] = arduinoTime & 0xff;
  arduinoTime = arduinoTime >> 8;
  dataToEspBuffer[PACKET_TO_ESP_SIZE - 4] = arduinoTime & 0xff;
  arduinoTime = arduinoTime >> 8;
  dataToEspBuffer[PACKET_TO_ESP_SIZE - 5] = arduinoTime & 0xff;
  arduinoTime = arduinoTime >> 8;
  dataToEspBuffer[PACKET_TO_ESP_SIZE - 6] = arduinoTime & 0xff;

  uint16_t sum = 0;
  for (uint8_t i = 0; i < PACKET_TO_ESP_SIZE - 2; i++)
  {
    sum += dataToEspBuffer[i];
  }
  dataToEspBuffer[PACKET_TO_ESP_SIZE - 2] = (sum >> 8);
  dataToEspBuffer[PACKET_TO_ESP_SIZE - 1] = (sum & 0xff);

  //using Serial protocol to send data to ESP
  for (uint8_t i = 0; i < PACKET_TO_ESP_SIZE; i++)
  {
    Serial.write(dataToEspBuffer[i]);
  }
  //done
}
