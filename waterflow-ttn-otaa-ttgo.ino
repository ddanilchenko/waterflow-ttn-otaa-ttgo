/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include "Arduino.h"
#include <EEPROM.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h> //https://github.com/ElectronicCats/CayenneLPP

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= { 0x59, 0x2B, 0x03, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0xD0, 0x67, 0x16, 0x5C, 0xB7, 0xCE, 0x35, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xE6, 0x51, 0x54, 0xC4, 0x50, 0x88, 0xBA, 0x1B, 0x7F, 0xAA, 0x51, 0x3F, 0x13, 0x81, 0x0F, 0x54 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60 / 2;

#define NSS_GPIO    18
#define RESET_GPIO  23
#define DIO0_GPIO   26
#define DIO1_GPIO   33
#define DIO2_GPIO   32
//TTGO!

volatile int count = 0;
const unsigned int confirmedStep = 5;

const  lmic_pinmap lmic_pins = {
    .nss = NSS_GPIO,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RESET_GPIO, 
    .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

#define HALL_SENSOR_PIN_1 14 //14
#define HALL_SENSOR_PIN_2 34 //13

#define NUMBER_OF_HALL_SENSORS 2

#define EEPROM_RESET_CMD 0x42


//OLED pins
#define OLED_SDA 21
#define OLED_SCL 22 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


const int hallSensorPin[NUMBER_OF_HALL_SENSORS] = { HALL_SENSOR_PIN_1, HALL_SENSOR_PIN_2 };


const int EEPROM_START_ADDR = 0;

// count how many pulses!
volatile uint16_t pulses[NUMBER_OF_HALL_SENSORS];
volatile uint16_t prevPulses[NUMBER_OF_HALL_SENSORS];

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);


void IRAM_ATTR pulseCounter1()
{
  pulseIncrement(0);
}

void IRAM_ATTR pulseCounter2()
{
  pulseIncrement(1);
}

void pulseIncrement(int sensorNo) {
  pulses[sensorNo]++;
}

void savePulsesToEEPROM() {
  for(int i = 0 ; i < NUMBER_OF_HALL_SENSORS; ++i) {
    EEPROM.put(EEPROM_START_ADDR + i * sizeof(pulses[i]), pulses[i]);
  }
  EEPROM.commit();
}

void useInterrupt(boolean v) {
  if (v) {
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN_1), pulseCounter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN_2), pulseCounter2, FALLING);
  } else {
    detachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN_1));
    detachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN_2));
  }
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            if (LMIC.dataLen == 1 && LMIC.frame[LMIC.dataBeg] == EEPROM_RESET_CMD) {
              resetPulsesCounters();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void updatePrevPulses() {
  for(int i = 0 ; i < NUMBER_OF_HALL_SENSORS; ++i) {
    prevPulses[i] = pulses[i];
  }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      savePulsesToEEPROM();
      displayPulses();
      readSensorDataAndSend();
      updatePrevPulses();
      Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void displayPulses() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("PULSE COUNTER");
  
  display.setCursor(0, 20);
  display.print("Total1:");
  display.setCursor(50, 20);
  display.print(pulses[0]);

  display.setCursor(0, 30);
  display.print("Actual1:");
  display.setCursor(50, 30);
  display.print(pulses[0] - prevPulses[0]);

  
  display.setCursor(0, 40);
  display.print("Total2:");
  display.setCursor(50,40);
  display.print(pulses[1]);

  display.setCursor(0, 50);
  display.print("Actual2:");
  display.setCursor(50, 50);
  display.print(pulses[1] - prevPulses[1]);
  
  display.display();
}

void resetPulsesCounters() {
  Serial.println(F("resetting pulses counters..."));
  for(int i = 0 ; i < NUMBER_OF_HALL_SENSORS; ++i) {
    EEPROM.put(EEPROM_START_ADDR + i * sizeof(pulses[i]), 0);
    pulses[i] = prevPulses[i] = 0;
  }
  EEPROM.commit();
  Serial.println(F("done pulses counters"));
}

void initOLED() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

void setup() {
    Serial.begin(9600);
    initOLED();
    
    Serial.println(F("Starting"));
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 3 / 100);
    LMIC_setLinkCheckMode(0);   // Disable link check validation
    LMIC_setAdrMode(false);     // Disable ADR
    LMIC.dn2Dr = DR_SF9; 

    initSensor();
    displayPulses();
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void initHallSensors() {
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    int sensorPin = hallSensorPin[i];
    pinMode(sensorPin, INPUT);
    //digitalWrite(sensorPin, HIGH);
    EEPROM.get(EEPROM_START_ADDR + i * sizeof(pulses[i]), pulses[i]);
  }
}

void initEEPROM() {
  EEPROM.begin(sizeof(pulses));
}

void initSensor() {
  //Serial.println(F("InitSensor BEGIN"));

  initEEPROM();
  initHallSensors();
  updatePrevPulses();
  useInterrupt(true);

  sensorOff();
  Serial.println(F("InitSensor END"));
}

void sensorOn() {
}

void sensorOff() {
}

CayenneLPP lppBuffer(42);

void readSensorDataAndSend() {
  
  Serial.print(F("Pulses:"));
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    Serial.print(pulses[i]);
    Serial.print(" ");
  }
  
  Serial.println();
  Serial.println(F("start sending data"));
  lppBuffer.reset();
  
  for (int i = 0; i < NUMBER_OF_HALL_SENSORS; ++i) {
    lppBuffer.addLuminosity(i + 2, pulses[i]);
  }
  Serial.println(F("done building buffer"));
  int confirmed = count++ == confirmedStep;
  LMIC_setTxData2(1, lppBuffer.getBuffer(), lppBuffer.getSize(), confirmed);
  if (confirmed) {
    count = 0;
  }
  Serial.println(F("done sending data"));
}

void loop() {
    os_runloop_once();
}
