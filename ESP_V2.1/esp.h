#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <FS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>    

//#define PIN_BUTTON 0 //for v2.0
#define PIN_BUTTON 12  //for v2.1
#define PIN_LED 14

#define num_byte_data_from_arduino 24
#define num_byte_send_2_arduino 8 // 8+6
#define queue_size 10
#define debug false

#define f_front_rear_size 3
#define f_data_size 20
#define f_queue_size 99994     //f_data_size*num_data

const char* fileName="/test4.txt";
const long utcOffsetInSeconds = 3600*7; // UTC +7 

uint8_t data_buff[num_byte_data_from_arduino]={0};
uint8_t data_i=0;

bool isGetTime=true;

uint32_t lastPress = 0;
uint32_t lastGetTime=0;
uint32_t lastMqttReconnect=0;
uint32_t lastCollectionData=0;

const char mqtt_server[15] = "23.89.159.119"; 
const uint16_t mqtt_port = 1883;
char topic[35];
char espID[25];
char nameDevice[12];


WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


struct data
{
    uint16_t pm1=0;
    uint8_t dot_pm1=0;
    uint16_t pm25=0;
    uint8_t dot_pm25=0;
    uint16_t pm10=0;
    uint8_t dot_pm10=0;
	  uint16_t co=0;
	  uint8_t dot_co=0;
    uint8_t temp=0;
    uint8_t dot_temp=0;
    uint8_t hum=0;
    uint8_t dot_hum=0;
    uint32_t epoch_time=0;
};
data datas[queue_size];
uint8_t front=0;
uint8_t rear=0;

void mqtt_begin();
bool longPress();
void enQueue(data _a);
data deQueue();
bool isQueueFull();
bool isQueueEmpty();
bool f_deQueue(const char* _fileName);
bool f_checkQueue(uint8_t* _data,const char* _fileName);
bool f_view(uint8_t* _buffer, const char* _fileName);
void f_init(const char* _fileName);
bool f_enQueue(uint8_t* _data,const char* _fileName);

bool f_deQueue(const char* _fileName)
{
    File _f = SPIFFS.open(_fileName, "r+");
    if (_f)
    {
        _f.seek(0);
        uint8_t _front[f_front_rear_size];
        _f.read(_front, f_front_rear_size);// front in text file

        _f.seek(f_front_rear_size);
        uint8_t _rear[f_front_rear_size];
        _f.read(_rear, f_front_rear_size);// rear in text file

        uint32_t __front = 0;
        uint32_t __rear = 0;

        for (size_t i = 0; i < f_front_rear_size; i++)
        {
            __front += _front[i] << (8 * (f_front_rear_size - i - 1));
            __rear += _rear[i] << (8 * (f_front_rear_size - i - 1));
        }

        if (__front == __rear)
        {
            return false;// file empty
        }

        if(debug) Serial.print("f_deQueue ");
        if(debug) Serial.print(__front);
        if(debug) Serial.print(" ");
        if(debug) Serial.print(__rear);
        
        __front = (__front + f_data_size) % f_queue_size;

        if(debug) Serial.print(" -> ");
        if(debug) Serial.print(__front);
        if(debug) Serial.print(" ");
        if(debug) Serial.println(__rear);
        
        for (size_t i = 0; i < f_front_rear_size; i++)
        {
            _front[f_front_rear_size - i - 1] = __front % 256;
            __front /= 256;
        }
        _f.seek(0);
        _f.write(_front, f_front_rear_size);// new front in text file

        _f.close();
        return true;// deQueue
    }
    else
    {
        return false;// text file is not exists
    }
}

bool f_checkQueue(uint8_t* _data,const char* _fileName)
{
    File _f = SPIFFS.open(_fileName, "r");
    if (_f)
    {
        _f.seek(0);
        uint8_t _front[f_front_rear_size];
        _f.read(_front, f_front_rear_size);// front in text file

        _f.seek(f_front_rear_size);
        uint8_t _rear[f_front_rear_size];
        _f.read(_rear, f_front_rear_size);// rear in text file

        uint32_t __front = 0;
        uint32_t __rear = 0;

        for (size_t i = 0; i < f_front_rear_size; i++)
        {
            __front += _front[i] << (8 * (f_front_rear_size - i - 1));
            __rear += _rear[i] << (8 * (f_front_rear_size - i - 1));
        }

        if (__front == __rear)
        {
            return false;// file empty
        }

        _f.seek(__front);
        _f.read(_data,f_data_size);// data
        _f.close();
        
        return true;
    }
    else
    {
        return false;// text file is not exists
    }   
}

bool f_view(uint8_t* _buffer, const char* _fileName)
{
    File _f = SPIFFS.open(_fileName, "r");
    if (_f)
    {
        _f.read(_buffer, 2 * f_front_rear_size);
        _f.close();
        return true;
    }
    else
    {
        return false;
    }   
}

void f_init(const char* _fileName)
{
    if (f_queue_size%f_data_size!=0)
    {
        return;//
    }

    File _f;

    if (SPIFFS.exists(_fileName)==false)
    {
        _f = SPIFFS.open(_fileName, "w");
        uint8_t front_rear[f_front_rear_size] = { 0 };
        front_rear[f_front_rear_size - 1] = 2 * f_front_rear_size;

        _f.seek(0);
        _f.write(front_rear, f_front_rear_size);
        _f.seek(f_front_rear_size);
        _f.write(front_rear, f_front_rear_size);

        _f.close();
    }
}

bool f_enQueue(uint8_t* _data,const char* _fileName)
{
    File _f = SPIFFS.open(_fileName, "r+");
    if (_f)
    {
        _f.seek(0);
        uint8_t _front[f_front_rear_size];
        _f.read(_front, f_front_rear_size);

        _f.seek(f_front_rear_size);
        uint8_t _rear[f_front_rear_size];
        _f.read(_rear, f_front_rear_size);

        uint32_t __front = 0;
        uint32_t __rear = 0;

        for (size_t i = 0; i < f_front_rear_size; i++)
        {
            __front += _front[i] << (8 * (f_front_rear_size - i - 1));
            __rear += _rear[i] << (8 * (f_front_rear_size - i - 1));
        }

        _f.seek(__rear);
        _f.write(_data, f_data_size);

        if (__front == (__rear + f_data_size) % f_queue_size)
        {
            __front = (__front + f_data_size) % f_queue_size;
            for (size_t i = 0; i < f_front_rear_size; i++)
            {
                _front[f_front_rear_size - i - 1] = __front % 256;
                __front /= 256;
            }
            _f.seek(0);
            _f.write(_front, f_front_rear_size);
        }

        if(debug) Serial.print("f_deQueue ");
        if(debug) Serial.print(__front);
        if(debug) Serial.print(" ");
        if(debug) Serial.print(__rear);
                
        __rear = (__rear + f_data_size) % f_queue_size;

        if(debug) Serial.print(" -> ");
        if(debug) Serial.print(__front);
        if(debug) Serial.print(" ");
        if(debug) Serial.println(__rear);
        
        for (size_t i = 0; i < f_front_rear_size; i++)
        {
            _rear[f_front_rear_size - i - 1] = __rear % 256;
            __rear /= 256;
        }
        _f.seek(f_front_rear_size);
        _f.write(_rear, f_front_rear_size);
    
        _f.close();
        
        return true;
    }
    else
    {
        return false;
    }
}

bool isQueueEmpty()
{
    if(front==rear)
        return true;
    else
        return false;
}

bool isQueueFull()
{
    if(front==(rear+1)%queue_size)
        return true;
    else
        return false;
}

void enQueue(data _a)
{
    if(isQueueFull())
    {
        //if queue is full, save data to file
        data save_data = deQueue();
        uint8_t _data[f_data_size]={0,0,0,0,save_data.temp,save_data.dot_temp,save_data.hum,save_data.dot_hum,save_data.pm1/256,save_data.pm1%256,save_data.dot_pm1,save_data.pm25/256,save_data.pm25%256,save_data.dot_pm25,save_data.pm10/256,save_data.pm10%256,save_data.dot_pm10,save_data.co/256,save_data.co%256,save_data.dot_co};
        uint32_t epoch_t = save_data.epoch_time;
        _data[3] = epoch_t & 0xff;
        epoch_t  = epoch_t >> 8;
        _data[2] = epoch_t & 0xff;
        epoch_t  = epoch_t >> 8;
        _data[1] = epoch_t & 0xff;
        epoch_t  = epoch_t >> 8;
        _data[0] = epoch_t & 0xff;
        f_enQueue(_data,fileName);        
    }
    datas[rear]=_a;
    rear=(rear+1)%queue_size;    
}

data deQueue()
{
    // chi goi deQueue khi queue is not empty           
    uint8_t _b=front;
    front=(front+1)%queue_size;
    
    if(isQueueEmpty())
    {
        uint8_t f_data[f_data_size];
        if (f_checkQueue(f_data,fileName))
        {
            if(f_deQueue(fileName))
            {
                data _data;
                _data.epoch_time = (f_data[0]<<24) + (f_data[1]<<16) + (f_data[2]<<8) + f_data[3];
                _data.temp       = f_data[4];
                _data.dot_temp   = f_data[5];
                _data.hum        = f_data[6];
                _data.dot_hum    = f_data[7];
                _data.pm1        = (f_data[8]<<8)+f_data[9];
                _data.dot_pm1    = f_data[10];
                _data.pm25       = (f_data[11]<<8)+f_data[12];
                _data.dot_pm1    = f_data[13];
                _data.pm10       = (f_data[14]<<8)+f_data[15];
                _data.dot_pm1    = f_data[16];
				        _data.co         = (f_data[17]<<8)+f_data[18];
                _data.dot_co     = f_data[19];
                enQueue(_data);
            }
        }
    }
    
    return datas[_b];    
}

bool longPress()
{
    if (millis() - lastPress > 3000 && digitalRead
    (PIN_BUTTON) == 0) 
    { 
        return true;
    } 
    else if (digitalRead(PIN_BUTTON) == 1) 
    {
        lastPress = millis();
    }
    return false;
}

void mqtt_begin()
{
    uint8_t macAddess[6];
    WiFi.macAddress(macAddess);
//    uint32_t macAddessDecimal= macAddess[0]<<40 + macAddess[1]<<32 + macAddess[2]<<24 + macAddess[3]<<16 + macAddess[4]<<8+macAddess[5];
    uint32_t macAddessDecimalTail = macAddess[3]*256*256 + macAddess[4]*256 + macAddess[5];
    uint32_t macAddessDecimalHead = macAddess[0]*256*256 + macAddess[1]*256 + macAddess[2];
//    sprintf(topic,"/SPARC/ESP_%06X_%06d/",macAddessDecimal,(int)(macAddessDecimal&0xFFFFF));
    sprintf(topic,"/SPARC/AirSENSE_%6X%6X", macAddessDecimalHead, macAddessDecimalTail);
    sprintf(espID,"AirSENSE_%6X%6X", macAddessDecimalHead, macAddessDecimalTail);
    sprintf(nameDevice,"%6X%6X", macAddessDecimalHead, macAddessDecimalTail);
    
//    sprintf(nameDevice,"ESP_%06X",macAddessDecimal);
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.connect(espID);
}
