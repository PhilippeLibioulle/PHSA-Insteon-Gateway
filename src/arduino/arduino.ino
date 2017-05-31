#include <ArduinoJson.h>
#include <avr/wdt.h>
#include <Ethernet.h>
#include <LiquidCrystal.h>
#include <Network.h>
#include <PubSubClient.h>
#include <QueueArray.h>
#include "SoftwareSerial.h"
#include <SPI.h>
#include <Timer.h>

// HW pinout
const int LCD_E_PIN PROGMEM = 2;                                             // LCD
const int RESET_BTN_PIN PROGMEM = 3;                                         // goes to LOW when someone press on the button and then goes to HIGH when button is released, otherwise pull-down to GND
const int SD_CARD_PIN PROGMEM = 4;                                           // SD card on ethernet shield, not used
const int LCD_DB4_PIN PROGMEM = 5;                                           // LCD
const int LCD_DB5_PIN PROGMEM = 6;                                           // LCD
const int LCD_DB6_PIN PROGMEM = 7;                                           // LCD
const int LCD_DB7_PIN PROGMEM = 8;                                           // LCD
const int LCD_RS_PIN PROGMEM = 9;                                            // LCD
const int INSTEON_TX PROGMEM = 16;                                           // TX to INSTEON PLM
const int INSTEON_RX PROGMEM = 17;                                           // RX from INSTEON PLM

// Network
const byte mac[] = { 0xDE, 0xED, 0xBE, 0xBB, 0xFC, 0xAC };                   // Arduino's MAC address
IPAddress ip(192, 168, 22, 221);                                              // Arduino's IP address
IPAddress server(192, 168, 22, 127);                                          // MQTT broker's address  (Orechestrator)
EthernetClient ethClient;

// MQTT 
PubSubClient client(ethClient); 
#define mqttUser "kitto"    
#define mqttPsw "kitty"          
#define mqttClientPrefix "GLX"                                               // Prefix to use any MQTT publication/subscription 
#define mqttClientLocation "GROUNDFLOOR"                                     // Second part of the client identifier
#define mqttClientUID "002"                                                  // Last part of the client identifier
#define mqttClientStatusTopic "Status"                                       // Topic to be used to publish device status 
#define mqttClientFaultTopic "Fault"                                         // Topic to be used to publish/subscribe to Faults
const int mqttInterval PROGMEM = 300;                                        // determines how often the system will report to MQTT broker (ie. every mqttInterval * mainLoopDelay ms )
int mqttIntervalCnt = 0;                                                     // local variable used to count down
int hasSubscribed = -1;                                                      // -1 = unknow, 1 = has subscribed to TODO topic, 0 = has not been able to subscribe
int isConnectedToBroker = -1;                                                // 1 when connected, -1 = unknown, 0 = unable to connected
#define INSTEONGATEWAY "PLM"

// Insteon 
SoftwareSerial InsteonSerial(INSTEON_RX, INSTEON_TX, true);                 // This is what we use to send/receive Insteon commands using the home automation shield
const int PLMtimeOut PROGMEM = 1000;                                        // in millisec 
int inByte[26]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};      // for storing incoming serial bytes
int outByte[26]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};     // for storing ougoing serial bytes
const int zeroByte PROGMEM= 0x00;                                           // I use this because if I do "Serial.write (0);" instead I get a compile error
const int startByte PROGMEM = 0x02;                                         // Every Insteon message begins with this
const int msgLength PROGMEM = 30;                                           // Used to assign an expected length for the message... starts out high so that we don't prematurely trigger an end-of-message
int i = 0;                                                                  // Looping variable for incoming messages
int j = 0;                                                                  // Looping variable for outgoing messages

// Display and log
const int mode PROGMEM = 1;                                                 // 0 = normal, 1 = verbose
LiquidCrystal lcd (LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN, LCD_DB5_PIN ,LCD_DB6_PIN, LCD_DB7_PIN); 
const int LCD_ROWS = 2;  
const int LCD_COLUMNS = 16;  

// Local queue
volatile QueueArray <byte> cmdQueue;                                        // each command received from MQTT broker is stored here
byte* incoming;                                                             // an incoming message to route to Insteon network

// Manual RESET button
volatile boolean isResetRequested = 0;                                      // this one will change when button triggers an interrupt

// Logic
const int mainLoopDelay = 500;                                              // a fixed delay within main loop, in ms
void(* resetFunc) (void) = 0;

// Initialization 
void setup()
{
  wdt_disable();                                                   // disable watchdog
  Serial.begin(19200);                                             // open console port 
  Serial.println(F("Begin of setup"));  
  // HW setup
  pinMode(SD_CARD_PIN, OUTPUT);                                    // always good to disable it, if it was left 'on' or you need init time
  digitalWrite(SD_CARD_PIN, HIGH);                                 // to disable SD card since we do not use it  
  pinMode (RESET_BTN_PIN, INPUT);                                  
  InsteonSerial.begin(19200);                                      // open another serial port for Insteon PLM
  cmdQueue.setPrinter (Serial);                                    // setup local queue
  lcd.begin(LCD_COLUMNS, LCD_ROWS);                                // set up the LCD's number of columns and rows:
  testLCD();                                                       // just a welcome message
  
  // Network and MQTT setup
  client.setServer(server, 1883);
  client.setCallback(MQTTBrokerCallback);
  Ethernet.begin(mac, ip);
  Serial.print(F("Current IP is : "));
  Serial.print(Ethernet.localIP());  
  Serial.print(F(" - MQTT broker IP is : "));
  Serial.println(server);  
  
  enableInterruptOnResetButton();                                  // from here, interrupts are trapped 
  delay(1500);  // allow hardware to sort itself out  
  Serial.println(F("End of setup")); Serial.println();      
}

// Main loop

void loop()
{  
  // Part 1 : react to reset request
  if (isResetRequested)
  {
     Serial.println(F("Someone pushed on the button to reset this device")); 
     routeGatewayStatusMessageToBroker();  
     wdt_enable(WDTO_1S); //enable watchdog, will fire in 1 second   
     delay(5000); 
     Serial.println(F("this message should never appear")); 
  }  
  
  // Part 2 : deal with messages originating from Insteon PLM
  
  while (InsteonSerial.available() > 0)       
  {      
     if (receiveMessageFromPLM() == 0) // message received and compliant with message type definition
     {   
        Serial.println();    
        if (inByte[1] == 0x50 && ( (inByte[8] & 0b11100000) == 192) && inByte[9] != 6 ) // we do not support all message types currently, just group broadcast messages
        {
           displayInsteonGroupBroadcast();         
           routeGroupBroadcastMessageToBroker();   
           Serial.println(); 
        }
        else
        {           
           Serial.println(F("Message type not currently supported."));  // See http://www.madreporite.com/insteon/receiving.html
           Serial.println(); 
        }
     };          
  };
  
  // Part 3 : deal with messages originating from MQTT Broker. Those messages are waiting in a local queue

  if (hasSubscribed < 1)
  {
     subscribeToToDoLists();     // from here, callbacks are trapped   
  }
  while (!cmdQueue.isEmpty ())
  {
     if (parseMessage() == 0)
     {
        sendMessageToInsteonDevice();  
     };
  }
   
  // Part 4 : report device status to MQTT broker
  
  if (mqttIntervalCnt == 0)
  { 
      Serial.println(F("Reporting gateway status to MQTT Broker..."));  
      routeGatewayStatusMessageToBroker();        
      mqttIntervalCnt = mqttInterval;
      Serial.println();
  }    
  else
  {                   
      mqttIntervalCnt = mqttIntervalCnt - 1;      
  }       
  // Take some rest
  delay(mainLoopDelay / 2 ); 
  client.loop();      
  delay(mainLoopDelay / 2);    
}

// Parse a message stored in the local queue

int parseMessage()
{
  byte b;
  int hex_digit_to_read = 2;
  bool reading_hex = true;
  byte hex = 0;
  for(int j=0;j<26;j++) outByte[j] = 0xFF;
  while (!cmdQueue.isEmpty ())
  { 
     b = cmdQueue.dequeue(); 
     if (b == ':')   // delimiter between each set of 2 characters 
     {
         hex = 0;
         hex_digit_to_read = 2;
         reading_hex = true;
         continue;
     };
     if (hex_digit_to_read > 0)  
     {
        hex = (hex << 4) | CharToHex(b);
        hex_digit_to_read--;
     };
     if (reading_hex && hex_digit_to_read == 0)
     {
        reading_hex = false;
        if (hex == 0x02)   // end of message
        {
          hex_digit_to_read = 2;
          reading_hex = true;
          hex = 0;
          j = 0;
          return 0;
        }
        else
        {
          outByte[j] = hex;
          Serial.print(hex, HEX);
          Serial.print(' ');
          j++;
        };
     };     
  };
}
// Send commands to Insteon devices

int sendMessageToInsteonDevice()
{
   Serial.print(F("Sending command to Insteon devices : "));   
   sendCmd(2, false);
   sendCmd(98, false);
   sendCmd(outByte[1], true);   
   sendCmd(outByte[2], true);   
   sendCmd(outByte[3], true);   
   sendCmd(15, false);   
   sendCmd(outByte[4], false);   
   sendCmd(outByte[5], false); 
   Serial.println();         
}

void sendCmd(byte b, bool isHex)
{
   if (isHex) { Serial.print(b, HEX); } else { Serial.print(b); };
   Serial.print(F("-"));
   InsteonSerial.write(b);
}

// Gather bytes sent by Insteon PLM in order to get a well structured message

int receiveMessageFromPLM()
{
   long start_time = millis();  
   int currentIndex = 0;    
   for(int j=0;j<26;j++) inByte[j] = 0;
   byte currentByte;        
   while (true)
   {
      if ((millis() - start_time) > PLMtimeOut)  // we should get a complete message in a short period of time
      {
         displayError1(); 
         return 1;  
      };
      if (InsteonSerial.available() > 0)
      {
         if (currentIndex == 0) 
         {
            Serial.print(F("### New message entering : ")); 
         }        
         currentByte = InsteonSerial.read();
         inByte[currentIndex] = currentByte;
         displayRawData(currentByte);
         if (currentIndex == 0 && currentByte != startByte)  // a new message should always start with the specified start byte
         {
            // displayError2(currentByte); 
            return 2;
         };           
         if (currentIndex > 11)   // message looks longer than expected
         {            
            return 4; 
         };       
         if (currentIndex == 10) // message has been received as expected
         {
            return 0;  // full message received
         }; 
         currentIndex = currentIndex + 1;  // just keep going with parsing
      };
   };   
}

// Route a command issued by MQTT Broker to Insteon devices

void routeGroupBroacastToInsteon()
{   
   // displayInsteonOutgoingGroupBroadcast();
   for (int i=0;i<26;i++)
   {     
      if ( outByte[i] == 0xFF) { break;}
      Serial.print(InsteonSerial.write(outByte[i]));
   }        
}

// Reset button management

void resetAll()
{
    Serial.println(F("Someone pushed on the button to reset this device")); 
    displayInfo("Reset requested");
    routeGatewayStatusMessageToBroker();	
    wdt_enable(WDTO_1S); //enable watchdog, will fire in 1 second   
    delay(5000); 
    Serial.println(F("This message should never appear... unless this board is a zombie"));
}
void enableInterruptOnResetButton()
{
    isResetRequested = 0;
    attachInterrupt(1, onResetRequested, CHANGE);
}
void onResetRequested()
{
    detachInterrupt(1);
    isResetRequested = 1;   
}

//
// MQTT related functions
//
// NOTE : MQTT_MAX_PACKET_SIZE = 128 bytes.. therefore not more than 100 for the payload !!!!!!!
//        unless you change it in /Arduino/libraries/pubSubClient/src/PubSubClient.h

// Route broadcasts to MQTT broker (topic = GLI/XX-YY-ZZ/WishList)

void routeGroupBroadcastMessageToBroker()
{
   char topic[30];
   strcpy(topic, "GLI");  
   strcat(topic, "/");
   strcat(topic, byteToHexaString(inByte[2]));
   strcat(topic, "-");
   strcat(topic, byteToHexaString(inByte[3]));
   strcat(topic, "-");
   strcat(topic, byteToHexaString(inByte[4]));
   strcat(topic,"/");
   strcat(topic, "WishList");   
   char payload[100];
   strcpy(payload, "{"); 
   strcat(payload,"\n"); 
   strcat(payload,"\"Group\": ");
   strcat(payload, byteToString(inByte[7]));  
   strcat(payload, ",");
   strcat(payload,"\n"); 
   strcat(payload,"\"Command\": ");
   strcat(payload, byteToHexaString(inByte[9]));
   strcat(payload, ",");
   strcat(payload,"\n"); 
   strcat(payload,"\"Parameters\": ");
   strcat(payload, byteToString(inByte[10]));
   strcat(payload,"\n"); 
   strcat(payload,"}");    
   publishMsg( topic, payload);  
}

// Route gateway status to MQTT broker (topic = GLI/Gateway/Status)

void routeGatewayStatusMessageToBroker()
{
   char topic[30];
   strcpy(topic, "GLI");  
   strcat(topic, "/");  
   strcat(topic, "Gateway");   
   strcat(topic,"/");
   strcat(topic, "Status");   
   char payload[100];
   strcpy(payload, "{"); 
   strcat(payload,"\n"); 
   strcat(payload,"\"ManualReset\": ");
   strcat(payload,"\n"); 
   if (isResetRequested == 1) { strcat(payload, "Yes");} else {strcat(payload, "No");};     
   strcat(payload,"\n"); 
   strcat(payload, ",");
   strcat(payload,"\n"); 
   strcat(payload,"\"LocalQueueLevel\": ");
   // strcat(payload, intToString(cmdQueue.count));  
   strcat(payload,"}"); 
   publishMsg( topic, payload);  
}

// Subscribe to the MQTT broker to get list of commands to forward to Insteon devices (topic = GLI/Gateway/ToDo)

void subscribeToToDoLists()   
{ 
  if (connectToBroker() == true)
  {   	
	   char topic[50];
	   strcpy(topic, "GLI");  
     strcat(topic, "/");
     strcat(topic, "Gateway");  
     strcat(topic,"/");
     strcat(topic, "ToDo");  
	   client.subscribe(topic);                  // otherwise subscriptions will growth forever..
     if (client.subscribe(topic) == true)
     {
        isConnectedToBroker = 1;    
        hasSubscribed = 1;  
        Serial.print(F("Registred to MQTT broker as a subscriber for the following topic: ")); 
        Serial.println(topic);
     }
     else
     {     
        Serial.println(F("Not registred to MQTT broker as a subscriber")); 
        hasSubscribed = 0;
     }    
     client.loop();          
  }
  else
  {
    isConnectedToBroker = 0;
	  Serial.println(F("Cannot subscribe to any topic since connection to MQTT broker is not established"));
  } 
}

// This function will be invoked by MQTT broker each time a message is added to GLI/Gateway/ToDo queue

void MQTTBrokerCallback(char* subscribedTopic, byte* payload, unsigned int msgLength)
{
  Serial.print(F("New message received from MQTT broker. Topic = "));
  Serial.print(subscribedTopic);
  Serial.print(F(", Length = "));
  Serial.println(msgLength);
  cmdQueue.enqueue('0');
  cmdQueue.enqueue('2');
  for (int i=0;i<msgLength;i++)
  {     
     cmdQueue.enqueue(payload[i]);   // store msg in local Queue     
  }  
  Serial.println();    
}

// Report to MQTT broker

void publishMsg(const char* topic, const char* payload)  
{
  if (connectToBroker() == true)
  {          
     if (client.publish( topic, payload ) == true)
     {
        isConnectedToBroker = 1;    
        Serial.print(F("Message sent to MQTT broker using the following topic "));  
        Serial.println(topic);
     }
     else
     {     
        Serial.print(F("Message NOT sent to MQTT broker using the following topic "));  
        Serial.println(topic);
        isConnectedToBroker = 0;
     }    
     client.loop();       
  }
}

// Build the client identifier

String buildClientIdentifier()
{
  String data = mqttClientPrefix;
  data+="_";
  data+= mqttClientLocation;
  data+="_";
  data+= mqttClientUID; 
  return data; 
}

// Manage connection with MQTT broker

int connectToBroker() 
{
  Serial.println(F(""));        
  Serial.print(F("Connecting to network and to MQTT Broker... "));    
  char tempBuffer[200];
  buildClientIdentifier().toCharArray(tempBuffer,200);
  if (client.connect(tempBuffer, mqttUser, mqttPsw ) == true)
  {        
    Serial.print(F("connected as ")); 
    Serial.println(tempBuffer);
  } 
  else
  {
     switch (client.state())
     {
        case -4:
          Serial.println(F("MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time"));
          break;
        case -3:
          Serial.println(F("MQTT_CONNECTION_LOST - the network connection was broken"));
          break;
        case -2:
          Serial.println(F("MQTT_CONNECT_FAILED - the network connection failed"));
          break;
        case -1:
          Serial.println(F("MQTT_DISCONNECTED - the client is disconnected cleanly"));
          break;
        case 0:
          break;
        case 1:
          Serial.println(F("MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT"));
          break;
        case 2:
          Serial.println(F("MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier"));
          break;
        case 3:
          Serial.println(F("MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection"));
          break;
        case 4:
          Serial.println(F("MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected"));
          break;
        case 5:
          Serial.println(F("MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect"));
          break;
        default:
          Serial.print("failed, rc=");
          Serial.println(client.state()); 
          break;        
     }
  }
  return client.connected();    
}

//
// LCD management
//


void displayInsteonGroupBroadcast()
{
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print('G');
   lcd.setCursor(2,0);
   lcd.print(inByte[2], HEX);  
   lcd.print('-');
   lcd.print(inByte[3], HEX);   
   lcd.print('-');
   lcd.print(inByte[4], HEX);  
   lcd.setCursor(0,1);   
   lcd.print("CMD");
   lcd.setCursor(5,1);   
   lcd.print(inByte[9], HEX); 
   lcd.setCursor(8,1);
   lcd.print("TO ");
   lcd.setCursor(11,1);
   lcd.print(inByte[7]); 
   delay(1000);
}

void displayInsteonOutgoingGroupBroadcast( const char* cmd, const char* dest, const char* parms )
{
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print('I');
   lcd.setCursor(2,0);
   lcd.print('MQTT BROKER');
   lcd.print("CMD");
   lcd.setCursor(5,1);   
   lcd.print(cmd); 
   lcd.setCursor(8,1);
   lcd.print("TO ");
   lcd.setCursor(11,1);
   lcd.print(dest); 
   delay(1000);
}
void displayBrokerMessage()
{    
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print('X');   
   delay(500);   
}
void displayError(const String& code, const String& message)
{    
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print('E');
   lcd.setCursor(0,2);
   lcd.print(code);
   lcd.setCursor(0,1);
   lcd.print(message);
   delay(500);
}
void displayInfo(const String& info)
{
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print(info);
   delay(500);
}
void testLCD()
{
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Hello PHSA !");
   delay(500);
}
void displayRawData(byte b)
{
   if (mode == 1)
   {
      Serial.print(F("["));
      Serial.print(b, HEX);   
      Serial.print(F("]"));
   };
}
void displayError1()
{
   if (mode == 1)
   {
      Serial.print(F("An error (type 1) has occured while parsing message from PLM : communication has timed out"));   
   };
   displayError("1","Parsing");
}
void displayError2(byte currentByte)
{
   if (mode == 1) 
   { 
      Serial.print(F("An error (type 2) has occured while parsing message from PLM : ["));  
      Serial.print(currentByte, HEX);
      Serial.println(F("] skipped since it does not start with 0x20 as per the standard"));
   }; 
   displayError("2","Structure");
}



// Utility functions


char* byteToHexaString(byte b)
{
    char buff[4];
    snprintf(buff, 4, "%02X", b);
    return buff;
}

char* byteToString(byte b)
{
    char buff[4];
    snprintf(buff, 4, "%d", b);
    return buff;
}

char* intToString(int i)
{
    char buff[4];
    snprintf(buff, 4, "%d", i);
    return buff;
}

int doubleBytesToInt(byte high_byte, byte low_byte)
{
    return high_byte * 256 + low_byte; 
}

int StrToHex(char str[])
{
  return (int) strtol(str, 0, 16);
}

byte CharToHex(char c)
{
    byte out = 0;
    if( c >= '0' && c <= '9'){
        out = (byte)(c - '0');
    } 
    else if ( c >= 'A' && c <= 'F'){
        out = (byte) (c - 'A') + 10;
    }
    else if ( c >= 'a' && c <= 'f'){
        out = (byte) (c - 'a') + 10;
    }

    return out;
}
