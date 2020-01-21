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

//#define DMP
//#define MPU
#define ICM20948

#ifdef ICM20948
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
//#define USE_SPI       // Uncomment this to use SPI
#endif
#ifdef MPU
#include <quaternionFilters.h>
#include <MPU9250.h>
#endif // MPU
#ifdef DMP
#include <SparkFunMPU9250-DMP.h> // Include SparkFun MPU-9250-DMP library
#endif // DMP
#include <Wire.h> //Needed for I2C to GPS
#include <WiFi.h>
#include <WiFiUdp.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

#ifdef OLED
#include "SSD1306.h"
#endif // OLED

#ifdef ICM20948
#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0
#endif

#ifdef USE_SPI
ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif


// WiFi network name and password:
const char * networkName = "NETGEAR71";
const char * networkPswd = "chummyskates355";

#ifdef MPU
#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;  // Set up pin 13 led for toggling

#define I2Cclock 400000
#define I2Cport Wire
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

//MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
MPU9250 myIMU;
#endif // MPU

#ifdef DMP
MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class
#endif //DMP

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

unsigned int counter = 0;

#ifdef OLED
SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);
#endif

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

unsigned long _lastMpuReadTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
unsigned long _lastGpsReadTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

unsigned long _lastGpsTimeMillis = 0; //The computer time of the last GPS reading

bool MpuInitialized = false;

const char* formatPrefix = "{\"Type\":\"telem\",\"From\":\"%s\",\"To\":\"%s\",\"Time\":%lu";
const char* formatCompass = ",\"Orient\":{\"Mag\":%f}";
const char* formatGPS = ",\"Coord\":{\"Lat\":%ld,\"Lon\":%ld,\"Alt\":%ld}";

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
int _second = 0;
unsigned int _milliSecond = 0;

unsigned long _timeBetweenMpuReadsMs = 100;
unsigned long _timeBetweenGpsReadsMs = 1000;

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
#ifdef USE_SPI
    SPI_PORT.begin();
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

#ifdef ICM20948
    if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
    {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    //myGPS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX, 1000); //Set the I2C port to input ...
    myGPS.setNavigationFrequency(1); //Produce one solutions per second
    myGPS.setAutoPVT(true); //Tell the GPS to "send" each solution
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) 
    {
        Serial.println("Trying again...");
        delay(500);
    }
    else 
    {
        MpuInitialized = true;
    }

#endif
#ifdef MPU

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);

    if (c == 0x71) // WHO_AM_I should always be 0x71
    {
        Serial.println(F("MPU9250 is online..."));

        // Start by performing self test and reporting values
        myIMU.MPU9250SelfTest(myIMU.SelfTest);
        Serial.print(F("x-axis self test: acceleration trim within : "));
        Serial.print(myIMU.SelfTest[0], 1); Serial.println("% of factory value");
        Serial.print(F("y-axis self test: acceleration trim within : "));
        Serial.print(myIMU.SelfTest[1], 1); Serial.println("% of factory value");
        Serial.print(F("z-axis self test: acceleration trim within : "));
        Serial.print(myIMU.SelfTest[2], 1); Serial.println("% of factory value");
        Serial.print(F("x-axis self test: gyration trim within : "));
        Serial.print(myIMU.SelfTest[3], 1); Serial.println("% of factory value");
        Serial.print(F("y-axis self test: gyration trim within : "));
        Serial.print(myIMU.SelfTest[4], 1); Serial.println("% of factory value");
        Serial.print(F("z-axis self test: gyration trim within : "));
        Serial.print(myIMU.SelfTest[5], 1); Serial.println("% of factory value");

        // Calibrate gyro and accelerometers, load biases in bias registers
        //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    }
    else
        Serial.println(F("### MPU9250 is not online ###"));

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
        // Communication failed, stop here
        Serial.println(F("Magnetometer communication failed"));
        Serial.flush();
        //return;
    }

	// Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
        //  Serial.println("Calibration values: ");
        Serial.print("X-Axis factory sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[0], 2);
        Serial.print("Y-Axis factory sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[1], 2);
        Serial.print("Z-Axis factory sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
	// data to calculate bias and scale.
	//myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magbias[0]);
    Serial.println(myIMU.magbias[1]);
    Serial.println(myIMU.magbias[2]);

    //Serial.println("AK8963 mag scale (mG)");
    //Serial.println(myIMU.magScale[0]);
    //Serial.println(myIMU.magScale[1]);
    //Serial.println(myIMU.magScale[2]);
    //    delay(2000); // Add delay to see results before serial spew of data

    if (SerialDebug)
    {
        Serial.println("Magnetometer:");
        Serial.print("X-Axis sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[2], 2);
	}


#endif // MPU
	
#ifdef DMP
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
#endif //DMP
}

void ResetGps()
{
	myGPS.hardReset();
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
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

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

//*****************************************************************************
//
//*****************************************************************************
void buildOutString(char* buffer)
{
    unsigned int timeOffset = 0;

    //if we are between GPS readings, find the time elapsed since the last one
    if (!_haveGps)
        if(_lastGpsTimeMillis > 0)
            timeOffset = millis() - _lastGpsTimeMillis;

	sprintf(outBuffer, formatPrefix, fromId, toId, _milliSecond + timeOffset);

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
    //Serial.println("---");

    _haveCompass = false;
    _haveGps = false;
	
#ifdef ICM20948
    if (millis() - _lastMpuReadTime > _timeBetweenMpuReadsMs)
    {
        _lastMpuReadTime = millis(); //Update the timer

        if (myICM.dataReady())
        {
            // read MPU data
            myICM.getAGMT();                

            _compass = computeCompassHeading(myICM.agmt);
            _haveCompass = true;

            // printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
            // printScaledAGMT(myICM.agmt);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
            //delay(30);
        }
        else
        {
            Serial.println("Waiting for data");
            //delay(500);
        }
    }
#endif	
#ifdef DMP
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
                //initMpu();
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

                delay(100);
	        }
		}
	}
#endif // DMP
	
	//Query module only every second. Doing it more often will just cause I2C traffic.
	//The module only responds when a new position is available
    //if (false)
    if (millis() - _lastGpsReadTime > _timeBetweenGpsReadsMs)
    if (myGPS.getPVT())
    {
        _lastGpsReadTime = millis(); //Update the timer

        //Serial.println("--- gotPVT ---");

        if (!myGPS.isConnected())
        {
            Serial.println("###### isConnected() not ok ######");
            //return;
        }

        /*if (!myGPS.checkUblox())
        {
            Serial.println("###### checkUblox() not ok ######");
            return;
        }*/


        //return;

        latitude = myGPS.getLatitude();
		longitude = myGPS.getLongitude();
		altitude = myGPS.getAltitude();
		timeOfWeek = myGPS.getTimeOfWeek();

		//myGPS.processRTCMframe()

        unsigned int milliSecond = ((((myGPS.getHour() * 60) + myGPS.getMinute()) * 60) + myGPS.getSecond()) * 1000 + myGPS.getMillisecond();

        if (milliSecond == _milliSecond)
        {
            if (_milliSecond != 0)
            {
                _milliSecond = 0;
                //Serial.println("###### milliSecond not changed, resetting GPS ######");
                Serial.println("###### milliSecond not changed #####");
                //ResetGps();
                return;
            }
        }
        else
        {
            _milliSecond = milliSecond;
            _lastGpsTimeMillis = millis();
        }

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

    //Serial.println("---");

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

float computeCompassHeading(ICM_20948_AGMT_t agmt)
{
    float heading;
    float mx = agmt.mag.axes.x;
    float my = agmt.mag.axes.y;

    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);

    if (heading > PI) heading -= (2 * PI);
    else if (heading < -PI) heading += (2 * PI);
    else if (heading < 0) heading += 2 * PI;

    heading *= 180.0 / PI;

    return heading;
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val) {
    if (val > 0) {
        SERIAL_PORT.print(" ");
        if (val < 10000) { SERIAL_PORT.print("0"); }
        if (val < 1000) { SERIAL_PORT.print("0"); }
        if (val < 100) { SERIAL_PORT.print("0"); }
        if (val < 10) { SERIAL_PORT.print("0"); }
    }
    else {
        SERIAL_PORT.print("-");
        if (abs(val) < 10000) { SERIAL_PORT.print("0"); }
        if (abs(val) < 1000) { SERIAL_PORT.print("0"); }
        if (abs(val) < 100) { SERIAL_PORT.print("0"); }
        if (abs(val) < 10) { SERIAL_PORT.print("0"); }
    }
    SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt) {
    SERIAL_PORT.print("RAW. Acc [ ");
    printPaddedInt16b(agmt.acc.axes.x);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.acc.axes.y);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.acc.axes.z);
    SERIAL_PORT.print(" ], Gyr [ ");
    printPaddedInt16b(agmt.gyr.axes.x);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.gyr.axes.y);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.gyr.axes.z);
    SERIAL_PORT.print(" ], Mag [ ");
    printPaddedInt16b(agmt.mag.axes.x);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.mag.axes.y);
    SERIAL_PORT.print(", ");
    printPaddedInt16b(agmt.mag.axes.z);
    SERIAL_PORT.print(" ], Tmp [ ");
    printPaddedInt16b(agmt.tmp.val);
    SERIAL_PORT.print(" ]");
    SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
    float aval = abs(val);
    if (val < 0) {
        SERIAL_PORT.print("-");
    }
    else {
        SERIAL_PORT.print(" ");
    }
    for (uint8_t indi = 0; indi < leading; indi++) {
        uint32_t tenpow = 0;
        if (indi < (leading - 1)) {
            tenpow = 1;
        }
        for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
            tenpow *= 10;
        }
        if (aval < tenpow) {
            SERIAL_PORT.print("0");
        }
        else {
            break;
        }
    }
    if (val < 0) {
        SERIAL_PORT.print(-val, decimals);
    }
    else {
        SERIAL_PORT.print(val, decimals);
    }
}

void printScaledAGMT(ICM_20948_AGMT_t agmt) {
    SERIAL_PORT.print("Scaled. Acc (mg) [ ");
    printFormattedFloat(myICM.accX(), 5, 2);
    SERIAL_PORT.print(", ");
    printFormattedFloat(myICM.accY(), 5, 2);
    SERIAL_PORT.print(", ");
    printFormattedFloat(myICM.accZ(), 5, 2);
    SERIAL_PORT.print(" ], Gyr (DPS) [ ");
    printFormattedFloat(myICM.gyrX(), 5, 2);
    SERIAL_PORT.print(", ");
    printFormattedFloat(myICM.gyrY(), 5, 2);
    SERIAL_PORT.print(", ");
    printFormattedFloat(myICM.gyrZ(), 5, 2);
    SERIAL_PORT.print(" ], Mag (uT) [ ");
    printFormattedFloat(myICM.magX(), 5, 2);
    SERIAL_PORT.print(", ");
    printFormattedFloat(myICM.magY(), 5, 2);
    SERIAL_PORT.print(", ");
    printFormattedFloat(myICM.magZ(), 5, 2);
    SERIAL_PORT.print(" ], Tmp (C) [ ");
    printFormattedFloat(myICM.temp(), 5, 2);
    SERIAL_PORT.print(" ]");
    SERIAL_PORT.println();
}

void printMagCalOut(ICM_20948_AGMT_t dpEng)
{
    // Print the sensor data
    Serial.print("Raw:");

    Serial.print(dpEng.acc.axes.x);
    Serial.print(',');
    Serial.print(dpEng.acc.axes.y);
    Serial.print(',');
    Serial.print(dpEng.acc.axes.z);
    Serial.print(',');

    Serial.print(dpEng.gyr.axes.x);
    Serial.print(',');
    Serial.print(dpEng.gyr.axes.y);
    Serial.print(',');
    Serial.print(dpEng.gyr.axes.z);
    Serial.print(',');

    Serial.print(dpEng.mag.axes.x);
    Serial.print(',');
    Serial.print(dpEng.mag.axes.y);
    Serial.print(',');
    Serial.print(dpEng.mag.axes.z);
    Serial.println();
}

//*********************************************************


