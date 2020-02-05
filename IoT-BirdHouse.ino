#include <LoraMessage.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include "LowPower.h"
#include <Arduino.h>
#include <Wire.h>

// For the SHT30 temperature sensor
#include "ClosedCube_SHT31D.h"
ClosedCube_SHT31D sht3xd;
int temp;
int humidity;

// Data wire is plugged into port 2 on the Arduino
//#define ONE_WIRE_BUS 2
//#define ONE_WIRE_POWER 3
#define VOLTAGE_PIN A0
#define VOLTAGE_MOSFET_PIN 8 // This pin controls the MOSFET used to enable the voltage devider for voltage reads

// For the distance measurement
#include <VL6180X.h>
VL6180X sensor;
const int numReadings = 10;     // the number of readings for smoothing
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int sleepcycles = 111;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 2     // pin 13 LED is not used, because it is connected to the SPI port

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { XXXXX };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { XXXXX };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = XXXXX ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
static osjob_t initjob;

byte sendBuff[14];

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = 0, //LMIC_UNUSED_PIN,
  .rst = 0,
  .dio = {4, 5, 7},
};

void onEvent (ev_t ev) {
  int i, j;
  switch (ev) {
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
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin, LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg], HEX);
        i = (LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i > 10) {
          i = 10;   // maximum number of BLINKs
        }
        for (j = 0; j < i; j++)
        {
          digitalWrite(LedPin, HIGH);
          delay(200);
          digitalWrite(LedPin, LOW);
          delay(400);
        }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      // Serial.print("Time done: ");
      // Serial.println(millis());
      delay(50);  // delay to complete Serial Output before Sleeping

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    //LMIC_setTxData2(1, (uint8_t*) buffer, 2 , 0);

    Serial.println(F("Send: "));
    //LMIC_setTxData2(1, loraMessage.getBytes(), loraMessage.getLength() , 0);
    LMIC_setTxData2(1, (uint8_t*) sendBuff, sizeof(sendBuff), 0);

  }
}

// initial job
static void initfunc (osjob_t* j) {
  // reset MAC state
  LMIC_reset();
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

int readVcc() {

  int readTime = millis();
  //float resistor1 = 47000;

  //float resistor2 = 15000;
  //denominator = (float)resistor2 / (resistor1 + resistor2);
  //float denominator = 0.24183666818;
  float denominator = 0.24783666818;

  // Enable the mosfet
  digitalWrite(VOLTAGE_MOSFET_PIN, HIGH);
  delay(2);

  // Read the analog voltage once to "flush" the capacitance
  analogRead(VOLTAGE_PIN);
  delay(1);

  //int value = analogRead(VOLTAGE_PIN);

  float reading = (analogRead(VOLTAGE_PIN) / 1024.0) * 1.1;
  reading = reading / denominator;

  // Turn of the mosfet
  digitalWrite(VOLTAGE_MOSFET_PIN, LOW);
  int result = reading * 100;

  readTime = millis() - readTime;
  Serial.print("readVCC Time: ");
  Serial.println(readTime);

  return result;
}


void transformTempResult(SHT31D result) {
  if (result.error == SHT3XD_NO_ERROR) {

    //Serial.println(result.t);
    temp = result.t * 100;
    humidity = result.rh * 100;

  } else {
    //Serial.print(": [ERROR] Code #");
    Serial.println(result.error);
  }
}

bool readTemp()
{
  int readTime = millis();

  bool state = false;
  //Serial.println("Requesting temp/hum");
  transformTempResult(sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50));
  //Serial.println("Got temp/hum");
  delay(10);
  //delay(250);
  state = true;

  readTime = millis() - readTime;
  Serial.print("readTemp Time: ");
  Serial.println(readTime);

  return state;
}

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println(F("Booting"));

  // Initilize the distance sensor
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Initialize the SHT30 temp sensor
  sht3xd.begin(0x45); // I2C address: 0x44 or 0x45

  // Set the analog reference to 1.1v intenal
  analogReference(INTERNAL);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(VOLTAGE_MOSFET_PIN, OUTPUT);

  // if LED is connected to pin 10, it has to be defined before any SPI initialization else
  // it will be used as SS (Slave Select) and controlled by the SPI module
  pinMode(LedPin, OUTPUT);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  Serial.println("Booting completed");

  // Fill buffer with initial data
  buildLoraData(255);
  delay(10);

  // Do initial send on boot
  do_send(&sendjob);    // Sent sensor values

}

bool buildLoraData(int i)
{
  int readTime = millis();
  // Perform a temp, hum, battery and distance measurement
  if (i == sleepcycles - 1 || i == 255)
  {
    int batt = readVcc();
    int distance = readDistance();

    sendBuff[12] = distance;
    sendBuff[13] = batt - 200;

    Serial.print("Diste: ");
    Serial.println(distance);
    Serial.print("Batt: ");
    Serial.println(batt);
  }

  if (i == 37 || i == 74 || i == sleepcycles - 1)
  {
    // Copy temperature reading one step back
    sendBuff[0] = sendBuff[4];
    sendBuff[1] = sendBuff[5];
    sendBuff[4] = sendBuff[8];
    sendBuff[5] = sendBuff[9];

    // Copy humidity reading one step back
    sendBuff[2] = sendBuff[6];
    sendBuff[3] = sendBuff[7];
    sendBuff[6] = sendBuff[10];
    sendBuff[7] = sendBuff[11];

    readTemp();
    sendBuff[8] = highByte(temp);
    sendBuff[9] = lowByte(temp);
    sendBuff[10] = highByte(humidity);
    sendBuff[11] = lowByte(humidity);

    Serial.print("Temp: ");
    Serial.println(temp);
    Serial.print("Hum: ");
    Serial.println(humidity);

  }
  else if (i == 255)
  {
    readTemp();
    sendBuff[8] = highByte(temp);
    sendBuff[9] = lowByte(temp);
    sendBuff[10] = highByte(humidity);
    sendBuff[11] = lowByte(humidity);

    // Copy initial temp reading to previous readings to have good initial data
    sendBuff[5] = sendBuff[9];
    sendBuff[4] = sendBuff[8];
    sendBuff[1] = sendBuff[5];
    sendBuff[0] = sendBuff[4];

    // Copy initial humidity reading to previous readings to have good initial data
    sendBuff[7] = sendBuff[11];
    sendBuff[6] = sendBuff[10];
    sendBuff[3] = sendBuff[7];
    sendBuff[2] = sendBuff[6];

    Serial.print("Temp: ");
    Serial.println(temp);
    Serial.print("Hum: ");
    Serial.println(humidity);
  }

  readTime = millis() - readTime;
  Serial.print("buildLoraMessage Time: ");
  Serial.println(readTime);

  return true;
}

int readDistance()
{
  int readTime = millis();
  //Serial.println("Requesting distance");
  for (int i = 0; i <= 10; i++)
  {
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = sensor.readRangeSingleMillimeters();
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average = total / numReadings;
    // send it to the computer as ASCII digits

    if (sensor.timeoutOccurred()) {
      Serial.print(" Distance sensor TIMEOUT");
    }
    delay(1);
  }

  readTime = millis() - readTime;
  Serial.print("Distance Read Time: ");
  Serial.println(readTime);

  return average;
}

//unsigned long time;

void loop() {

  while (sleeping == false)
  {
    os_runloop_once();
  }
  sleeping = false;
  for (int i = 0; i < sleepcycles; i++)
  {
    Serial.print("Sleeping: ");
    Serial.println(i);

    // get readings if we are in the right loop
    buildLoraData(i);

    delay(10);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
  }

  // Trigger LoRa Send
  do_send(&sendjob);    // Sent sensor values

}
