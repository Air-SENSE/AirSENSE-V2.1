#include "./esp.h"
void setup() 
{
    timeClient.begin();
    Serial.begin(9600);//txrx0
    pinMode(PIN_BUTTON, INPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    WiFi.mode(WIFI_STA);
    mqtt_begin();
    SPIFFS.begin();
    f_init(fileName);
}
void loop() 
{
    if (longPress())
    {
        digitalWrite(14, HIGH);
        WiFi.beginSmartConfig();       
    } 
    if(Serial.available() > 0)
    {
        data_buff[data_i]=Serial.read();
        if(data_buff[0]==66) 
        data_i++;
        if(data_i == num_byte_data_from_arduino)
        {
            data_i=0;
            if(data_buff[1]==77)
            {
                uint16_t check=0;
                for(uint8_t i=0;i<num_byte_data_from_arduino-2;i++)
                {
                    check+=data_buff[i];
                }
                uint16_t check2=data_buff[num_byte_data_from_arduino-2]*256+data_buff[num_byte_data_from_arduino-1];
                if(check==check2)
                {
                    //enQueue
                    data arduino_data;
                    arduino_data.temp = data_buff[2];
                    arduino_data.dot_temp = data_buff[3];
                    arduino_data.hum = data_buff[4];
                    arduino_data.dot_hum = data_buff[5];
          
                    arduino_data.pm1 = data_buff[6]*256+data_buff[7];
                    arduino_data.dot_pm1 = data_buff[8];
                    arduino_data.pm25 = data_buff[9]*256+data_buff[10];
                    arduino_data.dot_pm25 = data_buff[11];
                    arduino_data.pm10 = data_buff[12]*256+data_buff[13];
                    arduino_data.dot_pm10 = data_buff[14];
          
                    arduino_data.co = data_buff[15]*256+data_buff[16];
                    arduino_data.dot_co = data_buff[17];
                   
                    arduino_data.epoch_time = (data_buff[18] << 24) + (data_buff[19] << 16) + (data_buff[20] << 8) + (data_buff[21]);
                    enQueue(arduino_data);
                }
            }
        }
    }
    else if(WiFi.status() == WL_CONNECTED)
    {
        digitalWrite(14, LOW);    
        if(millis()-lastGetTime> 63000)
        {
          lastGetTime = millis();
          while(true)
          {
              if(millis()-lastGetTime>500) //sau 0.5s không nhận được data thì next luôn
                break;
                        //dateTime = NTPch.getNTPtime(7.0, 0);        
              if(timeClient.update())
              {
                 //if got time, send time to arduino
                 uint8_t time2arduino_buff[num_byte_send_2_arduino]={66,88,0,0,0,0,0,0};
                 uint32_t epochtime = timeClient.getEpochTime();
                 time2arduino_buff[5] = epochtime & 0xff;
                 epochtime = epochtime >> 8;
                 time2arduino_buff[4] = epochtime & 0xff;
                 epochtime = epochtime >> 8;
                 time2arduino_buff[3] = epochtime & 0xff;
                 epochtime = epochtime >> 8;
                 time2arduino_buff[2] = epochtime & 0xff;
//                 for(uint8_t i = 6; i < 12; i++)
//                 {
//                    time2arduino_buff[i] = nameDevice[i - 2];
//                 }
                 uint16_t checkSum  = 0;
                 for(uint8_t i=0;i<num_byte_send_2_arduino-2;i++)
                 {
                     checkSum += time2arduino_buff[i];
                 }
                 time2arduino_buff[num_byte_send_2_arduino-2] = checkSum >> 8;
                 time2arduino_buff[num_byte_send_2_arduino-1] = checkSum & 0xff;
                 //if(debug) Serial.print(" - time: ");
                 for(uint8_t i=0; i<num_byte_send_2_arduino; i++)
                 {
                     Serial.write(time2arduino_buff[i]);
                 }
                 //if(debug) Serial.println();
                 break;
               }
            }
        }     
         else if(isQueueEmpty()==false)
         {
            //if queue is not empty, publish data to server
            if(mqttClient.connected())
            {
                data send_data = deQueue();
                createAndPrintJSON(send_data, nameDevice);
                    mqttClient.loop();
            }
            else if(millis()-lastMqttReconnect>1000)
            {
                lastMqttReconnect=millis();
                //if(debug) Serial.println(" - mqtt reconnect ");
                mqttClient.connect(espID);
            }
        }
   }
}
void createAndPrintJSON(data _send_data,char *names)
 {
    float co    = _send_data.co + _send_data.dot_co*0.01;
    float hum   = _send_data.hum + _send_data.dot_hum*0.01;
    float tem   = _send_data.temp + _send_data.dot_temp*0.01;
    float pm1   = _send_data.pm1 + _send_data.dot_pm1*0.01;
    float pm2p5 = _send_data.pm25 + _send_data.dot_pm25*0.01;
    float pm10  = _send_data.pm10 + _send_data.dot_pm10*0.01;
    uint64_t realtimes = 0;
    realtimes |= _send_data.epoch_time ;
    DynamicJsonDocument doc(256);
    char mes[256] = {0};
    //JsonObject obj = doc.to<JsonObject>();
    JsonObject DATA = doc.createNestedObject("DATA"); 
    DATA["CO"]        = co; 
    DATA["HUM"]       = hum;
    DATA["PM1"]       = pm1;
    DATA["PM10"]      = pm10;
    DATA["PM2.5"]     = pm2p5;
    DATA["REALTIMES"] = realtimes;
    DATA["TEM"]       = tem;
    //JsonObject NAME = doc.createNestedObject("NAME"); 
    doc["NAME"] = names;
    //Serial.println();
    serializeJson(doc, mes);
    /*if (mqttClient.publish(topic, mes, true))
    {
      Serial.println(" -- Publish Done.");
    }*/
    mqttClient.publish(topic, mes, true);
 }
