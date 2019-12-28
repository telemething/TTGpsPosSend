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

//#include <quaternionFilters.h>
//#include <MPU9250.h>
#include <SparkFunMPU9250-DMP.h> // Include SparkFun MPU-9250-DMP library
#include <Wire.h> //Needed for I2C to GPS
#include <WiFi.h>
#include <WiFiUdp.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "SSD1306.h"

// WiFi network name and password:
const char * networkName = "NETGEAR71";
const char * networkPswd = "chummyskates355";

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

unsigned int counter = 0;

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

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

long lastTime1 = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
long lastTime2 = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

//*****************************************************************************
//
//*****************************************************************************
void connectToWiFi(const char * ssid, const char * pwd)
{
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//*****************************************************************************
//
//*****************************************************************************
void WiFiEvent(WiFiEvent_t event)
{
    switch(event)
	{
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

//*****************************************************************************
//
//*****************************************************************************
void initMpu()
{
    if (imu.begin() != INV_SUCCESS)
    {
        Serial.println("###### MPU Init Failed ######");
        return;
    }
    else
    {
        Serial.println("--- MPU Init OK ---");
    }
    
    // Use setSensors to turn on or off MPU-9250 sensors.
    // Any of the following defines can be combined:
    // INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_XYZ_COMPASS,
    // INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
    //if( INV_SUCCESS != imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
    if (INV_SUCCESS != imu.setSensors(INV_XYZ_COMPASS))
    {
        Serial.println("###### MPU setSensors Failed ######");
    }
    else
    {
        Serial.println("--- MPU setSensors OK ---");
    }
}

//*****************************************************************************
//
//*****************************************************************************
void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("TTGpsPosSend");

  //display.clear();
  //display.drawString(0, 0, "TTGpsPosSend");

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

  initMpu();
}

/*
{
	"Type":"telem",
	"From":"aDrone",
	"To":"to",
	"Time":"5248817670575246558",
	"Coord":
	{
		"Lat":47.468689019121889,
		"Lon":-121.76736511387465,
		"Alt":141.87001820094883
	},
	"Orient":
	{
		"Mag":60.5,
		"True":45.0,
		"X":0.0,
		"Y":0.38268345594406128,
		"Z":0.0,"W":0.92387950420379639
	},
	"Gimbal":
	{
		"X":0.38268345594406128,
		"Y":0.0,
		"Z":0.0,
		"W":0.92387950420379639
	},
	"MissionState":"unknown",
	"LandedState":"unknown"
}
*/

const char* formatPrefix = "{\"Type\":\"telem\",\"From\":\"%s\",\"To\":\"%s\",\"Time\":%lu";
const char* formatCompass = ",\"Orient\":{\"Mag\":%f}";
const char* formatGPS = "{,\"Coord\":{\"Lat\":%ld,\"Lon\":%ld,\"Alt\":%ld}";

char* fromId = "self";
char* toId = "*";
char tempBuffer[1024];
char outBuffer[1024];
float _compass = 0;
bool _haveCompass = false;
bool _haveGps = false;
long latitude = 0;
long longitude = 0;
long altitude = 0;
unsigned int timeOfWeek = 0;

//*****************************************************************************
//
//*****************************************************************************
void buildOutString(char* buffer)
{
	sprintf(outBuffer, formatPrefix, fromId, toId, timeOfWeek);

    if (_haveGps)
    {
        sprintf(tempBuffer, formatGPS, latitude, longitude, altitude);
        strcat(outBuffer, tempBuffer);
    }

    if (_haveCompass)
    {
        sprintf(tempBuffer, formatCompass, _compass);
        strcat(outBuffer, tempBuffer);
    }

    strcat(outBuffer, "}");
}

//*****************************************************************************
//
//*****************************************************************************
void loop()
{
    _haveCompass = false;
    _haveGps = false;

    if (millis() - lastTime1 > 1000)
    {
        lastTime1 = millis(); //Update the timer

        if (!imu.dataReady())
        {
            Serial.println("###### IMU data not ready ######");
        }
        else
        {
	        //Latch IMU values
	        //if (INV_SUCCESS != imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS))
            //if (INV_SUCCESS != imu.update(UPDATE_COMPASS))
            if (INV_SUCCESS != imu.updateCompass())
	        {
	            Serial.println("###### IMU update Failed ######");
                initMpu();
	        }
	        else
	        {
                _haveCompass = true;
	            //float accelX = imu.calcAccel(imu.ax); // accelX is x-axis acceleration in g's
	            //float accelY = imu.calcAccel(imu.ay); // accelY is y-axis acceleration in g's
	            //float accelZ = imu.calcAccel(imu.az); // accelZ is z-axis acceleration in g's

	            //float gyroX = imu.calcGyro(imu.gx); // gyroX is x-axis rotation in dps
	            //float gyroY = imu.calcGyro(imu.gy); // gyroY is y-axis rotation in dps
	            //float gyroZ = imu.calcGyro(imu.gz); // gyroZ is z-axis rotation in dps

	            //float magX = imu.calcMag(imu.mx); // magX is x-axis magnetic field in uT
	            //float magY = imu.calcMag(imu.my); // magY is y-axis magnetic field in uT
	            //float magZ = imu.calcMag(imu.mz); // magZ is z-axis magnetic field in uT

                _compass = imu.computeCompassHeading();

                //sprintf(outBuffer, "magHXYZ: %f, %f, %f, %f", compass, magX, magY, magZ);
      	        //sprintf(outBuffer, "compass: %f", _compass);
        	    //Serial.println(outBuffer);
	        }
		}
	}
    	
	//Query module only every second. Doing it more often will just cause I2C traffic.
	//The module only responds when a new position is available
	if (millis() - lastTime2 > 1000)
	{
		lastTime2 = millis(); //Update the timer

		latitude = myGPS.getLatitude();
		longitude = myGPS.getLongitude();
		altitude = myGPS.getAltitude();
		timeOfWeek = myGPS.getTimeOfWeek();

		_haveGps = true;

		//byte SIV = myGPS.getSIV();
		//Serial.print(F(" SIV: "));
		//Serial.print(SIV);
        //Serial.println();
	}

    if (_haveCompass | _haveGps)
    {
        buildOutString(outBuffer);
        Serial.println(outBuffer);

        if (connected)
        {
            //Send a packet
            udpOut.beginPacket(udpOutAddress, udpOutPort);
            udpOut.print(outBuffer);
            udpOut.endPacket();
        }
    }

	// if there's data available, read a packet
	int packetSize = udpIn.parsePacket();
	if (packetSize)
	{
		Serial.print("> RTCM: size: ");
		Serial.print(packetSize);
		Serial.print(", From: ");
		IPAddress remote = udpIn.remoteIP();
		
		for (int i = 0; i < 4; i++) 
        {
			Serial.print(remote[i], DEC);
			if (i < 3) 
	        {
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
}
