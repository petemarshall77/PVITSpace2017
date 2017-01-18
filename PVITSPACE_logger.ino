//gps-ultrasonicv-log-3.2

#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
int temp =32;
long cm;
byte dmm;
long mm;
long mm0;
long time0;
long time1;
long dtime;
long spaceTemp;
long pingTemp;
byte OnePing = 6;
byte TPing= A0;
byte TSpace=A1;
byte relay=5;
// -------- gps-ultrasonic-log-3.2.ino---------
//Code for November 22 launch.  ping time presented in percent of 340.29m/sec
// actually percent of time to travel 1 meter or 2939 microsec
// temps calculated in C then adjusted for transmission to be C+173
// did this to avoid negative numbers and keep the temp range in 1 byte size
//expect -50 to 40 deg C  transmitted as 123 to 213  if temp goes higher Byte 
// will roll over, so a temp reading of 000 is 83deg C
// -------- gps-ultrasonic-log-3.1.ino---------
//
// November 1 2015

//
// found maesurements wernt made unless gps lock, fixed by reading sencors even when no gps lock
//-----------------gps-ultrasonic-log-3.0.ino------------------
//April 23 2015
// add control for heater for the ultrasonic, using a relay

//-----------------gps-ultrasonic-log-2.0.ino------------------
//April 23, 2015
//add temperature measurements one at ping sensor one outside 
// transmit and log them

//-----------------gps-ultrasonic-log-1.1.ino-----------------------------
//  added transmitting the gps time stamp hour, minute, seconds


//-----------------gps-ultrasonic-log-1.0.ino-----------------------------
// Modified by Dr. K of Rolling Robots Inc. for PVIT Space Balloon Launch April 2015
// this is minimum working code that logs GPS and Time returned by ultrasonic experiment
// Transmits via xbee the distance in mm returned by the ultrasonic experiment
// use xbee-rx.ino toe receive the data at base station xbee/arduino


// Ladyada's logger modified by Bill Greiman to use the SdFat library
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS Shield
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada
// Fllybob added 10 sec logging option
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false  

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Set the pins used
#define chipSelect 10
#define ledPin 13

File logfile;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

// blink out an error code
void error(uint8_t errno) {
  /*
  if (SD.errorCode()) {
   putstring("SD error: ");
   Serial.print(card.errorCode(), HEX);
   Serial.print(',');
   Serial.println(card.errorData(), HEX);
   }
   */
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup() {
  pinMode(relay, OUTPUT); // pin to control relay for heater
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(9600);
  Serial.println("\r\nUltimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, 11, 12, 13)) {
    //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

    
  Serial.println("Ready!");
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

long ping(byte pingPin)
{
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration;
  long cm;


  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

 // duration is in microseconds.  Sound travels about  .034cm in 1 microsecond.  .034 = 1/29
// the time traveled is twice the distance so divide by 2.

  cm=duration/29/2;
  
  return duration;
}

float tempSensor(byte tempPin){
  // read the input on analog pin 0:
  int sensorValue = analogRead(tempPin);
  float Vtemp=sensorValue*5000.0/1024;
  float Tc= ((10.888 - sqrt((-10.888)*(-10.888) + 4 * 0.00347 * (1777.3-Vtemp)))/(-0.00347 * 2)) + 30;
  
  return Tc;
}

void loop() {
  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    
    // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA 
    // will clear the received flag and can cause very subtle race conditions if
    // new data comes in before parse is called again.
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    // Sentence parsed! 
    //Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return;
    }

    // Rad. lets log it!
   // Serial.println("Log");

    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
        error(4);
    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))   logfile.flush();
 
  //Serial.println("done logging");
  }
   // Rad. lets log the ultrasonic experiment data and time stamp!
   // also send the time and ultrasonic ping time out through the xbee
    temp = GPS.hour;
    Serial.write(temp);  //send via xbee
    delay(20);
    String hourStr =  String(temp, DEC);
    temp = GPS.minute;
    Serial.write(temp);   //send via xbee
     delay(20);
    String minuteStr =  String(temp, DEC);
    temp = GPS.seconds;
    Serial.write(temp);   //send via xbee
     delay(20);
    String secondStr =  String(temp, DEC); 
    time1=ping(OnePing);
    temp=100*time1/2939;
    //Serial.println(temp, DEC);
    Serial.write(temp);   //send via xbee
     delay(20);
    String timeStr = String(time1, DEC);
    spaceTemp=tempSensor(TSpace);
    temp=spaceTemp-100+273;
    Serial.write(temp);   //send via xbee
     delay(20);
    String tempSpace = String(spaceTemp, DEC);
    pingTemp=tempSensor(TPing);
    temp=pingTemp-100+273;
    Serial.write(temp);   //send via xbee
     delay(20);
    String tempPing = String(pingTemp, DEC);
    String tempStr = hourStr+":"+minuteStr+":"+secondStr+","+timeStr+","+tempSpace+","+tempPing;
    // Serial.println(tempStr);  hourStr+":"+minuteStr+":"+secondStr+" ping-time= "+timeStr+
    char charBuf[50];
    tempStr.toCharArray(charBuf, 50); 
   logfile.write(charBuf);
   delay(1900);  //wait so we only do this once every 2 seconds or so
//  }

  if(pingTemp<=22){
    digitalWrite(relay, HIGH);
  }
  else{
    digitalWrite(relay, LOW);
  }

}


/* End code */

