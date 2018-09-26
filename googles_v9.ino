#include <Adafruit_NeoPixel.h>

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include <Adafruit_NeoPixel.h>
#ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.
 #include <avr/power.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>

/*=========================================================================
 *  FACTORYRESET_ENABLE       0 - No Action
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     0  

    #define PIN                     6
    #define NUMPIXELS               32
/*=========================================================================*/

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN);

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


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

uint8_t  mode   = 0, // Current animation effect
         offset = 0; // Position of spinny eyes
uint32_t color  = 0xFF0000; // Start red
uint32_t white  = 0xFFFFFF; // White
uint8_t  brightness = 255;  // upto 255
uint32_t prevTime;
uint32_t rainbow[7]={0x0000FF, 0x00FF00, 0xFF00FF, 0xFFFF00, 0xAA00AA, 0x222200, 0x005555};
uint8_t  rainbowCount = 7; 
uint8_t  i;
uint8_t  b;
uint8_t  l;
uint8_t  col;
uint8_t  currentSelection = '_';
uint8_t  newSelection = '_';
uint32_t colour;
uint32_t t;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  // turn off neopixel
  pixels.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // off
  }
  pixels.show();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));

  #ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.
    if(F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  //pixels.begin();
  //prevTime = millis();
  
}

/**
void resetAll() {
    //
    // Reset all LED's
    //
    pixels.clear();
    //for(i=0; i<32; i++) pixels.setPixelColor(i, 0);
    prevTime = t;
    pixels.show();
}
*/

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  /* Wait for new data to arrive */
  //uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  uint8_t len = readPacket(&ble, 5);
  
  if (len > 0) 
      
      {  
         newSelection = packetbuffer[1];
      }

      
         
      /* Got a packet! */
      printHex(packetbuffer, len);

      // Reset!
      if (newSelection == 'R' || currentSelection == 'R') {
        uint8_t red = packetbuffer[2];
        uint8_t green = packetbuffer[3];
        uint8_t blue = packetbuffer[4];
        Serial.print ("RGB #");
        if (red < 0x10) Serial.print("0");
        Serial.print(red, HEX);
        if (green < 0x10) Serial.print("0");
        Serial.print(green, HEX);
        if (blue < 0x10) Serial.print("0");
        Serial.println(blue, HEX);
    
        for(uint8_t i=0; i<NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(red,green,blue));
        }
        pixels.show(); // This sends the updated pixel color to the hardware.
      }

      // Color
      if (newSelection == 'C' || currentSelection == 'C') {
        uint8_t red = packetbuffer[2];
        uint8_t green = packetbuffer[3];
        uint8_t blue = packetbuffer[4];
        Serial.print ("RGB #");
        if (red < 0x10) Serial.print("0");
        Serial.print(red, HEX);
        if (green < 0x10) Serial.print("0");
        Serial.print(green, HEX);
        if (blue < 0x10) Serial.print("0");
        Serial.println(blue, HEX);
    
        for(uint8_t i=0; i<NUMPIXELS; i++) {
          pixels.setPixelColor(i, pixels.Color(red,green,blue));
        }
        pixels.show(); // This sends the updated pixel color to the hardware.
      }

      // Sparkle!
      if (newSelection == 'S' || currentSelection == 'S') {
        
        pixels.setBrightness(brightness);
        i = random(32);
        col = random(7);
        color=rainbow[col];
        pixels.setPixelColor(i, color);
        pixels.show();
        delay(3);
        pixels.setPixelColor(i, 0);
        //break;
       }

    // Glow white!
    // set all pixels to white
    if (newSelection == 'G' || currentSelection == 'G') {
      for(i=0; i<32; i++) { 
        pixels.setPixelColor(i, white);     
      }
      
      pixels.show();
      //for(l=0; l<4; l++) {
       //delay(1000);
       //offset++;
       // loop up to full brightness  
       for(b=0; b<100; b++) {
        pixels.setBrightness(b);      
        pixels.show();
        delay(5);
        } 
       //delay(200);
       //pixels.setBrightness(255);
       //delay(300);      
       // and then dim
       for(b=100; b>0; b--) {
        pixels.setBrightness(b);      
        pixels.show();
        delay(5);
        }
      //} 
      //resetAll;
      pixels.clear();
      pixels.show();
    }
    
      // Strobe!!! - or F for fit!
      if (newSelection == 'F' || currentSelection == 'F') {           
      // set all pixels to white
        for(i=0; i<32; i++) { 
          pixels.setPixelColor(i, white);     
        }
        for(b=0; b<10; b++) {
         pixels.setBrightness(255);
         pixels.show();
         delay(25);
         pixels.setBrightness(5); 
         pixels.show();
         delay(25);
        }
      pixels.clear();
      pixels.show();  
      }  

      // Chasing tails
      if (newSelection == 'T' || currentSelection == 'T') {           
        pixels.setBrightness(brightness);
        for(i=0; i<16; i++) {
          uint32_t c = 0;
          if(((offset + i) & 7) < 2) c = color; 
          pixels.setPixelColor(   i, c); // First eye
          pixels.setPixelColor(31-i, c); // Second eye (flipped)
        }
      pixels.show();
      offset++;
      delay(10);
      pixels.clear();
      pixels.show();  
      } 

      // Fill/Empty cylce
      if (newSelection == 'E' || currentSelection == 'E') {           
         pixels.setBrightness(brightness);
         for(i=0; i<16; i++) {
           //uint32_t c = 0;
           pixels.setPixelColor(   i, color); // First eye
           pixels.setPixelColor(31-i, color); // Second eye (flipped)
        pixels.show();
        delay(15);
      }
      
      for(i=0; i<16; i++) {
        //uint32_t c = 0;
        pixels.setPixelColor(   i, 0); // First eye
        pixels.setPixelColor(31-i, 0); // Second eye (flipped)
        pixels.show();
        delay(15);
      }
      offset++;
      pixels.clear();
      pixels.show();  
      }
      

  currentSelection = newSelection;
  
} 
