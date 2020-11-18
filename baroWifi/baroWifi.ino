//VERSION 1.2

#include <Senses_wifi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

SoftwareSerial db9(13,15);
String ssid="";
String password="";
// UDP variables

unsigned int destPort =50000;
unsigned int destPortSec=10110; //Port Miniplex 
IPAddress ipLocal;
IPAddress destIP;

char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,


WiFiUDP UDP;
WiFiUDP UDPSec;
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
int eePortUdpSec=210;
int eeMDAStatus=220;


bool bFirstRun=false;

//unsigned long previousWifi=0; // millis() returns an unsigned long.
unsigned long intervalWifi=20000;
unsigned long intervalBaro=2000;
unsigned long previousBaro = 0;
bool bWifiConnected=false;
long nBaudRate=4800;
bool bMDAStatus=false;




void setup() {


initdb9();
initValues();
initWifi();
  
  
  if (!bme.begin()) {
    db9.println("Could not find a valid BME680 sensor, check wiring!");

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
  char trameMDA[200];
  int nCS=0;
  


  if ((unsigned long)(currentMillis - previousBaro) >= intervalBaro)
 {
      if (! bme.performReading()) {
      db9.println("Failed to perform reading :(");
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
   // db9.println(tramePress);

    sprintf(trameTemp, "$IIXDR,C,%s,C,TempAir*", cTemp);
    nCS=getCheckSum(trameTemp);
    sprintf(trameTemp,"%s%02x",trameTemp,nCS);
   // db9.println(trameTemp);

    sprintf(trameHumidity,"$IIXDR,H,%s,P,Humidity*", cHumidity);
    nCS=getCheckSum(trameHumidity);
    sprintf(trameHumidity,"%s%02x",trameHumidity,nCS);
   // db9.println(trameHumidity);

    sprintf(trameWeather,"$IIXDR,P,%s,B,Barometer,C,%s,C,TempAir,H,%s,P,Humidity*",cPression,cTemp,cHumidity);
    nCS=getCheckSum(trameWeather);
    sprintf(trameWeather,"%s%02x",trameWeather,nCS);
    db9.println(trameWeather);
    sendUdpMsg(trameWeather,sizeof(trameWeather));
    sendUdpMsgSec(trameWeather,sizeof(trameWeather));

    if(bMDAStatus==1)
    {
    sprintf(trameMDA,"$IIMDA,0.0,I,%s,B,%s,C,0.0,C,%s,%s,0.0,C,0.0,T,0.0,M,0.0,N,0.0,N*",cPression,cTemp,cHumidity,cHumidity);
    nCS=getCheckSum(trameMDA);
    sprintf(trameMDA,"%s%02x",trameMDA,nCS);
    db9.println(trameMDA);
    sendUdpMsg(trameMDA,sizeof(trameMDA));
    sendUdpMsgSec(trameMDA,sizeof(trameMDA));
    }
    
     
  

  previousBaro=millis();
 }

//lecture serie

  
  String sBuffer;
  if(db9.available())
  {
    sBuffer=db9.readString();
    int strLen=sBuffer.length()+1;
    char db9Buffer[strLen];
    sBuffer.toCharArray(db9Buffer,strLen);
    decodeTrame(db9Buffer);
  }

    
  


 //Lecture reseau
 int packetSize = UDP.parsePacket();
  if (packetSize) {

    // read the packet into packetBufffer
    int n = UDP.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    db9.println(packetBuffer); 
    decodeTrame(packetBuffer);

  }

}




/**********************FONCTIONS*************************************/
void initdb9(void)
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

  db9.begin(nBaudRate);
  delay(2000);
  
  
  
}
void initWifi(void)
{
  char msg[50];
  db9.println();
  db9.println();
  EEPROM.begin(512);

  ssid=read_String(eeSSID);

  password=read_String(eePWD);

  db9.print("Connecting to ");
  db9.println(ssid);

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
      db9.print("Timeout WiFi dans ");
      db9.print(nTimeout);
      db9.println("s.");
      
    }
    b=!b;
    if(currentMillis-beginMillis>intervalWifi)
    {
      bWifiConnected=false;
      db9.print("Echec de la connexion au reseau ");
      db9.println(ssid);
      break;
    }
    else
    {
      bWifiConnected=true;
    }
    delay(500);
  }
     
  db9.println("");
  if(bWifiConnected)
  {
    ipLocal=WiFi.localIP();
    db9.println("WiFi connected");  
  db9.print("IP address: ");
  db9.println(ipLocal);
  db9.print("Broadcast IP address: ");
  db9.println(destIP);
  db9.print("Udp Port: ");
  db9.println(destPort);
  db9.print("Udp Port secondaire: ");
  db9.println(destPortSec);
  sprintf(msg,"$BARO,WiFiConnected,true,\n");

   UDP.begin(destPort);
   UDPSec.begin(destPortSec);
  }
  else
  {
    db9.println("WiFi not connected");  
    sprintf(msg,"$BARO,WiFiConnected,false,\n");
  }

  sendUdpMsg(msg,sizeof(msg));
  db9.println(msg);


}

void initValues(void)
{
  
  if(bWifiConnected)
    ipLocal=WiFi.localIP();
  
  EEPROM.begin(512);

  EEPROM.get(eeFirstRun,bFirstRun);
  db9.print("First Run: ");
  db9.println(bFirstRun);

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
    EEPROM.put(eeIntervalWifi,1000);
    EEPROM.put(eeFirstRun,bFirstRun);
    EEPROM.put(eePortUdpSec,10110);
    EEPROM.put(eeMDAStatus,false);

        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           
           
        } else {
          db9.println("ERROR! EEPROM commit failed");
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
  EEPROM.get(eePortUdpSec,destPortSec);
  EEPROM.get(eeMDAStatus,bMDAStatus);
  
 
  
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

void sendUdpMsgSec(char *sMsg,int msgSize)
{
    UDPSec.beginPacket(destIP, destPortSec); 
    UDPSec.write(sMsg, msgSize); 
    UDPSec.endPacket(); 
}

void decodeTrame(char sTrame[])
{
char *mot = NULL;
char msg[50];
char copyTrame[100];
strcpy(copyTrame,sTrame);

db9.print("copyTrame ");
db9.println(copyTrame);


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
           db9.println("EEPROM successfully committed");
            UDP.stop();
            WiFi.disconnect();
           initValues();
           initWifi();
           
        } else {
          db9.println("ERROR! EEPROM commit failed");
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
        db9.println(msg);
        
       }

       if(strcmp(mot,"getTemperatureOffset")==0)
       {
          char cOffset[5];
        EEPROM.begin(512);
        EEPROM.get(eeTempAddress,OffsetTemp);
        dtostrf(OffsetTemp, 4, 0, cOffset);
        sprintf(msg,"$BARO,TemperatureOffset,%s,\n",cOffset);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getIpAddress")==0)
       {
        char ip[20];
        sprintf(ip,"%s",WiFi.localIP().toString().c_str());
        sprintf(msg,"$BARO,IpAddress,%s,\n",ip);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getBroadcastAddress")==0)
       {
        char ip[20];
        sprintf(ip,"%s",destIP.toString().c_str());
        sprintf(msg,"$BARO,BroadcastAddress,%s,\n",ip);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
        
       }
       if(strcmp(mot,"getUdpPort")==0)
       {
        sprintf(msg,"$BARO,UdpPort,%d,\n",destPort);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getUdpPortSec")==0)
       {
        sprintf(msg,"$BARO,UdpPortSec,%d,\n",destPortSec);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getPeriod")==0)
       {
        db9.println("getPeriod");
        sprintf(msg,"$BARO,period,%d,\n",intervalBaro);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getWiFiStatus")==0)
       {
        db9.println("getWiFiStatus");
        if(bWifiConnected)
        {
          sprintf(msg,"$BARO,WiFiConnected,true,\n");
        }
        else
        {
          sprintf(msg,"$BARO,WiFiConnected,false,\n");
        }
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getSSID")==0)
       {
        char buf[100];
        unsigned int len=ssid.length();
        ssid.toCharArray(buf,len+1);
        db9.println("getSSID");
        sprintf(msg,"$BARO,SSID,%s,\n",buf);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getTimeoutWiFi")==0)
       {
        sprintf(msg,"$BARO,timeoutWiFi,%d,\n",intervalWifi);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getBaudRate")==0)
       {
        sprintf(msg,"$BARO,baudRate,%d,\n",nBaudRate);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"getMDAStatus")==0)
       {
        db9.println("getMDAStatus");
        sprintf(msg,"$BARO,MDAStatus,%d,\n",bMDAStatus);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
       }

       if(strcmp(mot,"setPressureOffset")==0)
       {
        db9.println("setPressureOffset");
        char enTete[10];
        char type[20];
          int data;

         db9.print("sTrame ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%17s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
       
        EEPROM.put(eePressAddress,data);
        
 
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           OffsetPress=data;
           char cOffset[5];
           dtostrf(OffsetPress, 4, 0, cOffset);
           sprintf(msg,"$BARO,PressureOffset,%s,\n",cOffset);
            sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
          
           
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }
       if(strcmp(mot,"setTemperatureOffset")==0)
       {
        db9.println(mot);
        db9.println("setTemperatureOffset");
        char enTete[10];
        char type[20];
          int data;

         db9.print("sTrame ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%20s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
       
        EEPROM.put(eeTempAddress,data);
        
 
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           OffsetTemp=data;
           char cOffset[5];
           dtostrf(OffsetTemp, 4, 0, cOffset);
           sprintf(msg,"$BARO,TemperatureOffset,%s,\n",cOffset);
            sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
          
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }
       if(strcmp(mot,"setBroadcastAddress")==0)
       {
        db9.println(mot);
        db9.println("setBroadcastAddress");
        char enTete[10];
        char type[20];
          int data1;
          int data2;
          int data3;
          int data4;

         db9.print("sTrame ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%19s,%d.%d.%d.%d,",&enTete,&type,&data1,&data2,&data3,&data4);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data1 ");
        db9.println(data1);
        db9.print("data2 ");
        db9.println(data2);
        db9.print("data3 ");
        db9.println(data3);
        db9.print("data4 ");
        db9.println(data4);
        
        IPAddress ip(data1,data2,data3,data4);
        EEPROM.put(eeDestIp,ip[0]);
        EEPROM.put(eeDestIp+1,ip[1]);
        EEPROM.put(eeDestIp+2,ip[2]);
        EEPROM.put(eeDestIp+3,ip[3]);
       
        
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");

           destIP=ip;
           char sIp[20];
        sprintf(sIp,"%s",destIP.toString().c_str());
        sprintf(msg,"$BARO,BroadcastAddress,%s,\n",sIp);
        sendUdpMsg(msg,sizeof(msg));
        db9.println(msg);
        
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

       if(strcmp(mot,"setUdpPort")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%10s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
        EEPROM.put(eePortUdp,data);
         
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           destPort=data;
           sprintf(msg,"$BARO,UdpPort,%d,\n",destPort);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

       if(strcmp(mot,"setUdpPortSec")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%13s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
        EEPROM.put(eePortUdpSec,data);
         
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           destPortSec=data;
           sprintf(msg,"$BARO,UdpPortSec,%d,\n",destPortSec);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

       if(strcmp(mot,"setMDAStatus")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%12s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
        EEPROM.put(eeMDAStatus,data);
         
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           bMDAStatus=data;
           sprintf(msg,"$BARO,MDAStatus,%d,\n",bMDAStatus);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

        if(strcmp(mot,"setPeriod")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%9s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
        EEPROM.put(eeintervalBaro,data);
         
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           intervalBaro=data;
           sprintf(msg,"$BARO,period,%d,\n",intervalBaro);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

       
        if(strcmp(mot,"setSSID")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        char data[100];
        String sData;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%7s,%s,",&enTete,&type,&data);
         sData=data;
         int vir=sData.lastIndexOf(",");
         if(vir!=-1)
         {
          sData=sData.substring(0,vir);
         }
     
         
        
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(sData);

        if(writeString(eeSSID,sData))
        {
          db9.println("EEPROM successfully committed");
           ssid=sData;
           char buf[100];
          unsigned int len=ssid.length();
          ssid.toCharArray(buf,len+1);
           sprintf(msg,"$BARO,SSID,%s,\n",buf);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        }
        else
        {
          db9.println("ERROR! EEPROM commit failed");
        }
          
       }

       if(strcmp(mot,"setPassword")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        char data[100];
        String sData;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%11s,%s,",&enTete,&type,&data);
         sData=data;
         int vir=sData.lastIndexOf(",");
         if(vir!=-1)
         {
          sData=sData.substring(0,vir);
         }
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(sData);

        if(writeString(eePWD,sData))
        {
          db9.println("EEPROM successfully committed");
           password=sData;
           char buf[100];
          unsigned int len=password.length();
          password.toCharArray(buf,len+1);
           sprintf(msg,"$BARO,password,%s,\n",buf);
           //sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        }
        else
        {
          db9.println("ERROR! EEPROM commit failed");
        }
     
          
       }

       if(strcmp(mot,"resetPassword")==0)
       {
        db9.println(mot);
        
        String pwd="NO";        
         
        if(writeString(eePWD,pwd)){
           db9.println("EEPROM successfully committed");
          
          password=pwd;
          char buf[100];
          unsigned int len=password.length();
          password.toCharArray(buf,len+1);
           sprintf(msg,"$BARO,password,%s,\n",buf);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

         if(strcmp(mot,"setWiFiConnect")==0)
       {
        db9.println("setWiFiConnect");
        initWifi();
       }

       if(strcmp(mot,"setTimeoutWiFi")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        unsigned int data;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%14s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
        EEPROM.put(eeIntervalWifi,data);
         
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           intervalWifi=data;
           sprintf(msg,"$BARO,timeoutWiFi,%d,\n",intervalWifi);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
         }
          
       }

        if(strcmp(mot,"setBaudRate")==0)
       {
        db9.println(mot);
        char enTete[10];
        char type[20];
        long data;

         db9.print("copyTrame2 ");
        db9.println(copyTrame);
         sscanf(copyTrame,"%5s,%11s,%d,",&enTete,&type,&data);
        db9.print("enTete ");
        db9.println(enTete);
        db9.print("type ");
        db9.println(type);
        db9.print("data ");
        db9.println(data);
        
        EEPROM.put(eeBaudRate,data);
         
        if (EEPROM.commit()) {
           db9.println("EEPROM successfully committed");
           nBaudRate=data;
           db9.end();
           initdb9();
           sprintf(msg,"$BARO,baudRate,%d,\n",nBaudRate);
           sendUdpMsg(msg,sizeof(msg));
            db9.println(msg);
        } else {
          db9.println("ERROR! EEPROM commit failed");
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
