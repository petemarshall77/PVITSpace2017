// GPS, temperature, and Geiger counter data logger for PVIT Space Balloon, 2017
//
// This is a modified version of the 2016 balloon software, gps-ultrasonic-log-3.2.ino, with
// - xBee transmission removed - data is just captured to SD card
// - Ping sensors removed - removed from payload
// - Geiger counters added
//  
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>

byte THeaterPin = A0;            // Temperature sensor inside heater area 
byte TSpacePin = A1;             // Temperature sensor outside environment
byte relayPin = 5;               // Heater relay
SoftwareSerial mySerial(19,18);  // GPS Serial
Adafruit_GPS GPS(&mySerial);

// Set up GPS device
#define GPSECHO  false           // don't echo GPS to serial, set to true for debugging
#define LOG_FIXONLY false        // log all GPS data, even if a fix is not available  
boolean usingInterrupt = false;  // not currently using an interrupt
void useInterrupt(boolean);      // Func prototype keeps Arduino 0023 happy

// Geiger counters
SoftwareSerial geiger1(15,14);   // Geiger counter #1
SoftwareSerial geiger2(17,16);   // Geiger counter #2

// Miscellaneous Pins
#define chipSelect 10            // Chipselect Pin (TODO verify this for ATMega)
#define ledPin 13

File logfile;

//----------------------------------------------------
// Read a Hex value and return the decimal equivalent
//-----------------------------------------------------
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 10;
  if (c <= 'F')
    return (c - 'A') + 10;
}

//-------------------------
// blink out an error code
//-------------------------
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    delay(2000);
  }
}


//--------------------------
// SETUP
//--------------------------
void setup() {

  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("\r\nPVITSPACE_logger started.");

  // Start the SD card interface
  pinMode(10, OUTPUT);  // must set the SS pin to output for SD library to work

  // see if the card is present and can be initialized:
  if (!SD.begin()) {
    Serial.println("SD card initialization failed!");
    error(2);
  }
  Serial.println("SD card initialized.");

  // Open the SD card file
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

  // Connect to the GPS at the desired rate
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // only need RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // update rate
  GPS.sendCommand(PGCMD_NOANTENNA);              // turn off antenna updates, if firmware permits

  useInterrupt(true); // use interrupts to read GPS data

  // Start the Gieger counter ports
  geiger1.begin(9600);
  geiger2.begin(9600);
  
  Serial.println("Ready!");
}

//----------------------------------------------------------------------------------------
// This interrupt is called once a millisecond. Looks for any new GPS data, and stores it
//----------------------------------------------------------------------------------------
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


//----------------------------------
// Turn interrupt usage on or off
//----------------------------------
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



//----------------------------------------------------------
// Read a temperature sensor
//----------------------------------------------------------
float tempSensor(byte tempPin){
  int sensorValue = analogRead(tempPin);
  float Vtemp=sensorValue*5000.0/1024;
  float Tc= ((10.888 - sqrt((-10.888)*(-10.888) + 4 * 0.00347 * (1777.3-Vtemp)))/(-0.00347 * 2)) + 30;
  
  return Tc;
}

//----------------------------------------------------------
// LOOP
//----------------------------------------------------------
void loop() {
  
  // Read GPS data if not using interrupt
  if (!usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO) // debug     
      if (c) Serial.print(c);
  }
  
  // Process a GPS sentence if a new one is received
  if (GPS.newNMEAreceived()) {
    char *stringptr = GPS.lastNMEA();
    if (!GPS.parse(stringptr))   // Can't parse? Leave it and wait for next sentence
      return;

    // Are we looking for a GPS fix?
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return;
    }

    // Log the sentence to SD card
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize))
      error(4);
    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA"))
      logfile.flush();
  }

  // Get the gieger counter data
  int geiger1count = 0;
  int geiger2count = 0;
  while (geiger1.available()) {
    char c = geiger1.read();
    geiger1count++;
  }
  while (geiger2.available()) {
    char c = geiger2.read();
    geiger2count++;
  }
  
  // Log the time and temperature data
  String hourStr =  String(GPS.hour, DEC);
  String minuteStr =  String(GPS.minute, DEC);
  String secondStr =  String(GPS.seconds, DEC); 
  String tempSpace = String(tempSensor(TSpacePin), DEC);
  String tempHeater = String(tempSensor(THeaterPin), DEC);
  String geiger1str = String(geiger1count, DEC);
  String geiger2str = String(geiger2count, DEC);
  String logStr = hourStr+":"+minuteStr+":"+secondStr+" -> "+tempSpace+","+tempHeater+","+geiger1str+","+geiger2str;
  char charBuf[50];
  logStr.toCharArray(charBuf, 50); 
  logfile.write(charBuf);

  delay(1900);  // slow down logging rate   TODO: re-architect this if geiger buffer overflows

  float heaterTemp = tempSensor(THeaterPin);
  if (heaterTemp <= 22.0) {
    digitalWrite(relayPin, HIGH);
  }
  else if (heaterTemp >= 23.0) {
    digitalWrite(relayPin, LOW);
  }

}

