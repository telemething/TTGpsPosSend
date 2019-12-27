/*
 * TTGpsPosSend
 */

/*
  Reading lat and long via UBX binary commands - no more NMEA parsing!
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query a Ublox module for its lat/long/altitude. We also
  turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic 
  dramatically.

  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We 
  do this so that we don't have to use floating point numbers.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <quaternionFilters.h>
#include <MPU9250.h>
#include <Wire.h> //Needed for I2C to GPS
#include <WiFi.h>
#include <WiFiUdp.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

// WiFi network name and password:
const char * networkName = "NETGEAR71";
const char * networkPswd = "chummyskates355";

uint8_t gpsI2Caddress = 0x42; //Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
//const char * udpAddress = "192.168.0.255";
//const int udpPort = 3333;
const char* udpOutAddress = "192.168.1.255";
const int udpOutPort = 45679;
const char* udpInAddress = "192.168.1.255";
const int udpInPort = 45678;

//Are we currently connected?
boolean connected = false;

// data buffer
const unsigned int inDataBufferSize = 1024;

// buffer
uint8_t inBuffer[inDataBufferSize];

//The udp library class
WiFiUDP udpOut;
WiFiUDP udpIn;

SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
		  udpOut.begin(WiFi.localIP(), udpOutPort);
		  udpIn.begin(WiFi.localIP(), udpInPort);
		  connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("TTGpsPosSend");

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  myGPS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX, 1000); //Set the I2C port to input ...

  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

/*
{
	"type": "pose",
	"id": "self",
	"tow": 123456789,
	"coord":
	{
		"lat":474685466,
		"lon":3077292823,
		"alt":138180
	}
}
*/

const char* poseFormat = "{\"type\": \"pose\",\"id\" : \"%s\", \"tow\" : %lu, \"coord\" : { \"lat\": %ld, \"lon\" : %ld, \"alt\" : %ld }}";
char* senderId = "self";
char outBuffer[1024];

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

	long latitude = myGPS.getLatitude();
	long longitude = myGPS.getLongitude();
	long altitude = myGPS.getAltitude();
	unsigned long timeOfWeek = myGPS.getTimeOfWeek();

	sprintf(outBuffer, poseFormat, senderId, timeOfWeek, latitude, longitude, altitude);

	//only send data when connected
	if (connected) 
	{
		//Send a packet
		udpOut.beginPacket(udpOutAddress, udpOutPort);
		udpOut.print(outBuffer);
		udpOut.endPacket();
	}

	Serial.println(outBuffer);
    
    //long latitude = myGPS.getLatitude();
    //Serial.print(F("> Pos: Lat: "));
    //Serial.print(latitude);

    //long longitude = myGPS.getLongitude();
    //Serial.print(F(" Long: "));
    //Serial.print(longitude);
    //Serial.print(F(" (degrees * 10^-7)"));

    //long altitude = myGPS.getAltitude();
    //Serial.print(F(" Alt: "));
    //Serial.print(altitude);
    //Serial.print(F(" (mm)"));

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }
  //else
  //{  // if there's data available, read a packet
	  int packetSize = udpIn.parsePacket();
	  if (packetSize)
	  {
		  Serial.print("> RTCM: size: ");
		  Serial.print(packetSize);
		  Serial.print(", From: ");
		  IPAddress remote = udpIn.remoteIP();
		  for (int i = 0; i < 4; i++) {
			  Serial.print(remote[i], DEC);
			  if (i < 3) {
				  Serial.print(".");
			  }
		  }
		  Serial.print(":");
		  Serial.println(udpIn.remotePort());

		  // read the packet into packetBufffer
		  udpIn.read(inBuffer, inDataBufferSize);
		  //Serial.println("Contents:");
		  //Serial.println(inBuffer);

		  Serial.println(">>>> Sending RTCM to I2C");
		  Wire.beginTransmission(gpsI2Caddress); 
		  Wire.write(inBuffer, packetSize);              
		  Wire.endTransmission();     
		  Serial.println("Sent RTCM to I2C <<<<");
	  }
  //}
}
