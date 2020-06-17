
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>



String ssid="";
String password="";
// UDP variables

unsigned int destPort =50000;
IPAddress ipLocal;
IPAddress destIP;

char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,


WiFiUDP UDP;
Adafruit_BME680 bme; // I2C

int OffsetPress=0; //enPa
int OffsetTemp=0; //en°C
int eePressAddress=0;
int eeTempAddress=10;
int eeDestIp=20;
int eePortUdp=30;
int eeFirstRun=40;
int eeIntervalWifi=41;
int eeintervalBaro=50;
int eeSSID=60;
int eePWD=110;
int eeBaudRate=200;


bool bFirstRun=false;

//unsigned long previousWifi=0; // millis() returns an unsigned long.
unsigned long intervalWifi=20000;
unsigned long intervalBaro=2000;
unsigned long previousBaro = 0;
bool bWifiConnected=false;
long nBaudRate=4800;




void setup() {


initSerial();
initValues();
initWifi();
  
  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


  
}



void loop() {

  unsigned long currentMillis = millis(); // grab current time
  float pressure=0;
  float temperature=0;  
  float humidity=0;
  char cPression[9];
   char cTemp[7];
   char cHumidity[5];
  char tramePress[50];
  char trameTemp[50];
  char trameHumidity[50];
  char trameWeather[100];
  int nCS=0;
  


  if ((unsigned long)(currentMillis - previousBaro) >= intervalBaro)
 {
      if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
      }


    pressure = bme.pressure+OffsetPress;
    dtostrf(pressure / 100000, 7, 5, cPression);

    temperature=bme.temperature+float(OffsetTemp/10);
    dtostrf(temperature, 5, 2, cTemp);

    humidity=bme.humidity;
    dtostrf(humidity, 5, 2, cHumidity);
   
    /*******Génération trame $IIXDR**********/
    sprintf(tramePress, "$IIXDR,P,%s,B,Barometer*", cPression);
    nCS=getCheckSum(tramePress);
    sprintf(tramePress,"%s%02x",tramePress,nCS);
   // Serial.println(tramePress);

    sprintf(trameTemp, "$IIXDR,C,%s,C,Temperature*", cTemp);
    nCS=getCheckSum(trameTemp);
    sprintf(trameTemp,"%s%02x",trameTemp,nCS);
   // Serial.println(trameTemp);

    sprintf(trameHumidity,"$IIXDR,H,%s,P,Humidity*", cHumidity);
    nCS=getCheckSum(trameHumidity);
    sprintf(trameHumidity,"%s%02x",trameHumidity,nCS);
   // Serial.println(trameHumidity);

    sprintf(trameWeather,"$IIXDR,P,%s,B,Barometer,C,%s,C,Temperature,H,%s,P,Humidity*",cPression,cTemp,cHumidity);
    nCS=getCheckSum(trameWeather);
    sprintf(trameWeather,"%s%02x",trameWeather,nCS);
    Serial.println(trameWeather);

    sendUdpMsg(trameWeather,sizeof(trameWeather));
     
  

  previousBaro=millis();
 }

//lecture serie

  
  String sBuffer;
  if(Serial.available())
  {
    sBuffer=Serial.readString();
    int strLen=sBuffer.length()+1;
    char serialBuffer[strLen];
    sBuffer.toCharArray(serialBuffer,strLen);
    decodeTrame(serialBuffer);
  }

    
  


 //Lecture reseau
 int packetSize = UDP.parsePacket();
  if (packetSize) {

    // read the packet into packetBufffer
    int n = UDP.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    Serial.println(packetBuffer); 
    decodeTrame(packetBuffer);

  }

}




/**********************FONCTIONS*************************************/
void initSerial(void)
{
  EEPROM.begin(512);

  EEPROM.get(eeFirstRun,bFirstRun);
   if(bFirstRun!=0) //init des valeurs par défaut
  {
    EEPROM.put(eeBaudRate,4800);
    EEPROM.commit();
          
  }

  EEPROM.get(eeBaudRate,nBaudRate);
  if(nBaudRate>115200||nBaudRate<0)
  {
    nBaudRate=4800;
  }

  Serial.begin(nBaudRate);
  delay(2000);
  
  
  
}
void initWifi(void)
{
  char msg[50];
  Serial.println();
  Serial.println();
  EEPROM.begin(512);

  ssid=read_String(eeSSID);

  password=read_String(eePWD);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  String NoPwd="NO";
  if(password.compareTo(NoPwd)==0)
  {
    WiFi.begin(ssid);
  }
  else
  {
  WiFi.begin(ssid,password);
  }
  //WiFi.begin(ssid);
  unsigned long beginMillis = millis();
  unsigned long currentMillis;// grab current time
  bool b=false;
  
  while (WiFi.status() != WL_CONNECTED) {
    
    
    currentMillis=millis();
    if(b)
    {
      
      int nDiff=currentMillis-beginMillis;
      int nTimeout=intervalWifi-nDiff;
      nTimeout=nTimeout/1000;
      Serial.print("Timeout WiFi dans ");
      Serial.print(nTimeout);
      Serial.println("s.");
      
    }
    b=!b;
    if(currentMillis-beginMillis>intervalWifi)
    {
      bWifiConnected=false;
      Serial.print("Echec de la connexion au reseau ");
      Serial.println(ssid);
      break;
    }
    else
    {
      bWifiConnected=true;
    }
    delay(500);
  }
     
  Serial.println("");
  if(bWifiConnected)
  {
    ipLocal=WiFi.localIP();
    Serial.println("WiFi connected");  
  Serial.print("IP address: ");
  Serial.println(ipLocal);
  Serial.print("Broadcast IP address: ");
  Serial.println(destIP);
  Serial.print("Udp Port: ");
  Serial.println(destPort);

          sprintf(msg,"$BARO,WiFiConnected,true,\n");
       
          
        
        
   UDP.begin(destPort);
  }
  else
  {
    Serial.println("WiFi not connected");  
    sprintf(msg,"$BARO,WiFiConnected,false,\n");
  }

  sendUdpMsg(msg,sizeof(msg));
  Serial.print(msg);


}

void initValues(void)
{
  
  if(bWifiConnected)
    ipLocal=WiFi.localIP();
  
  EEPROM.begin(512);

  EEPROM.get(eeFirstRun,bFirstRun);
  Serial.print("First Run: ");
  Serial.println(bFirstRun);

  if(bFirstRun!=0) //init des valeurs par défaut
  {
    
    
      IPAddress fullBroadcast(255,255,255,255);
      destIP=fullBroadcast;
    
    bFirstRun=0;

    String sSSID="none";
    String sPassword="NO";
    writeString(eeSSID,sSSID);
    writeString(eePWD,sPassword);
    
    EEPROM.put(eePressAddress,0);
    EEPROM.put(eeTempAddress,0);
    EEPROM.put(eeDestIp,destIP[0]);
    EEPROM.put(eeDestIp+1,destIP[1]);
    EEPROM.put(eeDestIp+2,destIP[2]);
    EEPROM.put(eeDestIp+3,destIP[3]);
    EEPROM.put(eePortUdp,50000);
    EEPROM.put(eeintervalBaro,2000);
    EEPROM.put(eeIntervalWifi,60000);
    EEPROM.put(eeFirstRun,bFirstRun);

        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           
           
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
    
   
  }
  
    EEPROM.get(eePressAddress,OffsetPress);
  EEPROM.get(eeTempAddress,OffsetTemp);
  EEPROM.get(eeDestIp,destIP[0]);
  EEPROM.get(eeDestIp+1,destIP[1]);
  EEPROM.get(eeDestIp+2,destIP[2]);
  EEPROM.get(eeDestIp+3,destIP[3]);
  EEPROM.get(eePortUdp,destPort);
  EEPROM.get(eeintervalBaro,intervalBaro);
  EEPROM.get(eeIntervalWifi,intervalWifi);
  
 
  
}

// Calcul le CRC de la trame NMEA
int getCheckSum(char *string) {
  int i;
  int XOR;
  int c;
  // Calculate checksum ignoring any $'s in the string
  for (XOR = 0, i = 0; i < strlen(string); i++) {
    c = (unsigned char)string[i];
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  return XOR;
}

void sendUdpMsg(char *sMsg,int msgSize)
{
    UDP.beginPacket(destIP, destPort); 
    UDP.write(sMsg, msgSize); 
    UDP.endPacket(); 
    
    
}

void decodeTrame(char sTrame[])
{
char *mot = NULL;
char msg[50];
char copyTrame[100];
strcpy(copyTrame,sTrame);

Serial.print("copyTrame ");
Serial.println(copyTrame);


mot=strtok(sTrame,",");
bool bNextOffsetPres=false;
if(strcmp(mot,"$BARO")==0)
  {
    while(mot!=NULL)
    {

      if(strcmp(mot,"init")==0)
      {
        
        EEPROM.put(eeFirstRun,1);

        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
            UDP.stop();
            WiFi.disconnect();
           initValues();
           initWifi();
           
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
      }
      
      if(strcmp(mot,"getPressureOffset")==0)
       {
        char cOffset[5];
        EEPROM.begin(512);
        EEPROM.get(eePressAddress,OffsetPress);
        dtostrf(OffsetPress, 4, 0, cOffset);
        sprintf(msg,"$BARO,PressureOffset,%s,\n",cOffset);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
        
       }

       if(strcmp(mot,"getTemperatureOffset")==0)
       {
          char cOffset[5];
        EEPROM.begin(512);
        EEPROM.get(eeTempAddress,OffsetTemp);
        dtostrf(OffsetTemp, 4, 0, cOffset);
        sprintf(msg,"$BARO,TemperatureOffset,%s,\n",cOffset);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getIpAddress")==0)
       {
        char ip[20];
        sprintf(ip,"%s",WiFi.localIP().toString().c_str());
        sprintf(msg,"$BARO,IpAddress,%s,\n",ip);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getBroadcastAddress")==0)
       {
        char ip[20];
        sprintf(ip,"%s",destIP.toString().c_str());
        sprintf(msg,"$BARO,BroadcastAddress,%s,\n",ip);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
        
       }
       if(strcmp(mot,"getUdpPort")==0)
       {
        sprintf(msg,"$BARO,UdpPort,%d,\n",destPort);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getPeriod")==0)
       {
        Serial.println("getPeriod");
        sprintf(msg,"$BARO,period,%d,\n",intervalBaro);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getWiFiStatus")==0)
       {
        Serial.println("getWiFiStatus");
        if(bWifiConnected)
        {
          sprintf(msg,"$BARO,WiFiConnected,true,\n");
        }
        else
        {
          sprintf(msg,"$BARO,WiFiConnected,false,\n");
        }
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getSSID")==0)
       {
        char buf[100];
        unsigned int len=ssid.length();
        ssid.toCharArray(buf,len+1);
        Serial.println("getSSID");
        sprintf(msg,"$BARO,SSID,%s,\n",buf);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getTimeoutWiFi")==0)
       {
        sprintf(msg,"$BARO,timeoutWiFi,%d,\n",intervalWifi);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"getBaudRate")==0)
       {
        sprintf(msg,"$BARO,baudRate,%d,\n",nBaudRate);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
       }

       if(strcmp(mot,"setPressureOffset")==0)
       {
        Serial.println("setPressureOffset");
        char enTete[10];
        char type[20];
          int data;

         Serial.print("sTrame ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%17s,%d,",&enTete,&type,&data);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(data);
        
       
        EEPROM.put(eePressAddress,data);
        
 
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           OffsetPress=data;
           char cOffset[5];
           dtostrf(OffsetPress, 4, 0, cOffset);
           sprintf(msg,"$BARO,PressureOffset,%s,\n",cOffset);
            sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
          
           
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }
       if(strcmp(mot,"setTemperatureOffset")==0)
       {
        Serial.println(mot);
        Serial.println("setTemperatureOffset");
        char enTete[10];
        char type[20];
          int data;

         Serial.print("sTrame ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%20s,%d,",&enTete,&type,&data);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(data);
        
       
        EEPROM.put(eeTempAddress,data);
        
 
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           OffsetTemp=data;
           char cOffset[5];
           dtostrf(OffsetTemp, 4, 0, cOffset);
           sprintf(msg,"$BARO,TemperatureOffset,%s,\n",cOffset);
            sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
          
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }
       if(strcmp(mot,"setBroadcastAddress")==0)
       {
        Serial.println(mot);
        Serial.println("setBroadcastAddress");
        char enTete[10];
        char type[20];
          int data1;
          int data2;
          int data3;
          int data4;

         Serial.print("sTrame ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%19s,%d.%d.%d.%d,",&enTete,&type,&data1,&data2,&data3,&data4);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data1 ");
        Serial.println(data1);
        Serial.print("data2 ");
        Serial.println(data2);
        Serial.print("data3 ");
        Serial.println(data3);
        Serial.print("data4 ");
        Serial.println(data4);
        
        IPAddress ip(data1,data2,data3,data4);
        EEPROM.put(eeDestIp,ip[0]);
        EEPROM.put(eeDestIp+1,ip[1]);
        EEPROM.put(eeDestIp+2,ip[2]);
        EEPROM.put(eeDestIp+3,ip[3]);
       
        
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");

           destIP=ip;
           char sIp[20];
        sprintf(sIp,"%s",destIP.toString().c_str());
        sprintf(msg,"$BARO,BroadcastAddress,%s,\n",sIp);
        sendUdpMsg(msg,sizeof(msg));
        Serial.print(msg);
        
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }

       if(strcmp(mot,"setUdpPort")==0)
       {
        Serial.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         Serial.print("copyTrame2 ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%10s,%d,",&enTete,&type,&data);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(data);
        
        EEPROM.put(eePortUdp,data);
         
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           destPort=data;
           sprintf(msg,"$BARO,UdpPort,%d,\n",destPort);
           sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }

        if(strcmp(mot,"setPeriod")==0)
       {
        Serial.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         Serial.print("copyTrame2 ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%9s,%d,",&enTete,&type,&data);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(data);
        
        EEPROM.put(eeintervalBaro,data);
         
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           intervalBaro=data;
           sprintf(msg,"$BARO,period,%d,\n",intervalBaro);
           sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }

       
        if(strcmp(mot,"setSSID")==0)
       {
        Serial.println(mot);
        char enTete[10];
        char type[20];
        char data[100];
        String sData;

         Serial.print("copyTrame2 ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%7s,%s,",&enTete,&type,&data);
         sData=data;
         int vir=sData.lastIndexOf(",");
         if(vir!=-1)
         {
          sData=sData.substring(0,vir);
         }
     
         
        
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(sData);

        if(writeString(eeSSID,sData))
        {
          Serial.println("EEPROM successfully committed");
           ssid=sData;
           char buf[100];
          unsigned int len=ssid.length();
          ssid.toCharArray(buf,len+1);
           sprintf(msg,"$BARO,SSID,%s,\n",buf);
           sendUdpMsg(msg,sizeof(msg));
            Serial.println(msg);
        }
        else
        {
          Serial.println("ERROR! EEPROM commit failed");
        }
          
       }

       if(strcmp(mot,"setPassword")==0)
       {
        Serial.println(mot);
        char enTete[10];
        char type[20];
        char data[100];
        String sData;

         Serial.print("copyTrame2 ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%11s,%s,",&enTete,&type,&data);
         sData=data;
         int vir=sData.lastIndexOf(",");
         if(vir!=-1)
         {
          sData=sData.substring(0,vir);
         }
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(sData);

        if(writeString(eePWD,sData))
        {
          Serial.println("EEPROM successfully committed");
           password=sData;
           char buf[100];
          unsigned int len=password.length();
          password.toCharArray(buf,len+1);
           sprintf(msg,"$BARO,password,%s,\n",buf);
           //sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
        }
        else
        {
          Serial.println("ERROR! EEPROM commit failed");
        }
     
          
       }

       if(strcmp(mot,"resetPassword")==0)
       {
        Serial.println(mot);
        
        String pwd="NO";        
         
        if(writeString(eePWD,pwd)){
           Serial.println("EEPROM successfully committed");
          
          password=pwd;
          char buf[100];
          unsigned int len=password.length();
          password.toCharArray(buf,len+1);
           sprintf(msg,"$BARO,password,%s,\n",buf);
           sendUdpMsg(msg,sizeof(msg));
            Serial.println(msg);
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }

         if(strcmp(mot,"setWiFiConnect")==0)
       {
        Serial.println("setWiFiConnect");
        initWifi();
       }

       if(strcmp(mot,"setTimeoutWiFi")==0)
       {
        Serial.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         Serial.print("copyTrame2 ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%14s,%d,",&enTete,&type,&data);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(data);
        
        EEPROM.put(eeIntervalWifi,data);
         
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           intervalWifi=data;
           sprintf(msg,"$BARO,timeoutWiFi,%d,\n",intervalWifi);
           sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }

        if(strcmp(mot,"setBaudRate")==0)
       {
        Serial.println(mot);
        char enTete[10];
        char type[20];
        long data;

         Serial.print("copyTrame2 ");
        Serial.println(copyTrame);
         sscanf(copyTrame,"%5s,%11s,%d,",&enTete,&type,&data);
        Serial.print("enTete ");
        Serial.println(enTete);
        Serial.print("type ");
        Serial.println(type);
        Serial.print("data ");
        Serial.println(data);
        
        EEPROM.put(eeBaudRate,data);
         
        if (EEPROM.commit()) {
           Serial.println("EEPROM successfully committed");
           nBaudRate=data;
           Serial.end();
           initSerial();
           sprintf(msg,"$BARO,baudRate,%d,\n",nBaudRate);
           sendUdpMsg(msg,sizeof(msg));
            Serial.print(msg);
        } else {
          Serial.println("ERROR! EEPROM commit failed");
         }
          
       }
       
      mot=strtok(NULL,",");
    }
   
    
  }


  
}


bool writeString(int add,String data)
{
  bool bRes=false;
  int _size = data.length();
  int i;
  for(i=0;i<_size;i++)
  {
    EEPROM.write(add+i,data[i]);
  }
  EEPROM.write(add+_size,'\0');   //Add termination null character for String Data
  bRes=EEPROM.commit();

  return bRes;
}
 
 
String read_String(int add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len=0;
  unsigned char k;
  k=EEPROM.read(add);
  while(k != '\0' && len<500)   //Read until null character
  {    
    k=EEPROM.read(add+len);
    data[len]=k;
    len++;
  }
  data[len]='\0';
  return String(data);
}
