#include <FastLED.h>

const int TRIG_PIN = 6;
const int ECHO_PIN = 7;
const int LED_PIN = 2;
const int DISTANCE_THRESHOLD = 20; // centimeters
CRGB color = CRGB::Red;
bool motion = true;
//bool fading = false;

#define NUM_LEDS  80
#define LED_PIN   3

CRGB leds[NUM_LEDS];

// variables will change:
float duration_us, distance_cm;

/**************************************************************************/
//Bluetooth

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the software serial bluefruit object

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/

void setup() {
  
  Serial.begin(9600);       // initialize serial port
  pinMode(TRIG_PIN, OUTPUT); // set arduino pin to output mode
  pinMode(ECHO_PIN, INPUT);  // set arduino pin to input mode
  pinMode(LED_PIN, OUTPUT);  // set arduino pin to output mode
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  
  while (!Serial);  // required for Flora & Micro
  delay(500);
  
  Serial.begin(115200);
  Serial.println(F("ESE 111 Adafruit Bluefruit Lab"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset. This happens pretty freqeuntly the first time you switch computers or change the sketch.\nPlease try re- uploading the code 1 or 2 more times."));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  //ble.info();

  char command[BUFSIZE+1];
  Serial.println(F("************************************************************"));
  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("************************************************************"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  Serial.println(F("******************************"));

  // Check BLE version
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println("\n ********BLE PAIRED SUCCESSFULLY. READY TO GO!!!********\n");
  Serial.println(F("************************************************************"));

  FastLED.clear();
}

void bluetooth() {
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  
  /* Wait for new data to arrive */
  if (len == 0) return;

  //Check if a button was pressed
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      // Turn off motion sensor
      if (buttnum == 1) {
         motion = !motion;
      }
    } else {
      Serial.println(" released");
    }
  }

  if (packetbuffer[1] == 'C') {
    uint8_t red_val = packetbuffer[2];
    uint8_t green_val = packetbuffer[3];
    uint8_t blue_val = packetbuffer[4];
    Serial.println ("Received RGB #");
    color = CRGB(red_val, green_val, blue_val);
    if (red_val < 0x10) Serial.print("0");
    Serial.println(red_val);
    if (green_val < 0x10) Serial.print("0");
    Serial.println(green_val);
    if (blue_val < 0x10) Serial.print("0");
    Serial.println(blue_val);
    Serial.println();
  }
}

void sensor() {
  if (motion) {
    // generate 10-microsecond pulse to TRIG pin
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  
    // measure duration of pulse from ECHO pin
    duration_us = pulseIn(ECHO_PIN, HIGH);
    // calculate the distance
    distance_cm = 0.017 * duration_us;
  
    if(distance_cm >= DISTANCE_THRESHOLD) {
      FastLED.clear();
      FastLED.show();
    }
    else {
      for(int i = 0;i<=NUM_LEDS;i++){ leds[i] = color; } FastLED.show();
      delay(10);
    }
  }
  else {
    for(int i = 0;i<=NUM_LEDS;i++){ leds[i] = color; } FastLED.show();
        delay(10);
  }
}

void loop() {
  sensor();
  bluetooth();
}
