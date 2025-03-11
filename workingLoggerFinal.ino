
/*

This demonstrates how to save the join information in to permanent memory
so that if the power fails, batteries run out or are changed, the rejoin
is more efficient & happens sooner due to the way that LoRaWAN secures
the join process - see the wiki for more details.

This is typically useful for devices that need more power than a battery
driven sensor - something like a air quality monitor or GPS based device that
is likely to use up it's power source resulting in loss of the session.

The relevant code is flagged with a ##### comment

Saving the entire session is possible but not demonstrated here - it has
implications for flash wearing and complications with which parts of the 
session may have changed after an uplink. So it is assumed that the device
is going in to deep-sleep, as below, between normal uplinks.

Once you understand what happens, feel free to delete the comments and 
Serial.prints - we promise the final result isn't that many lines.

*/

#if !defined(ESP32)
  #pragma error ("This is not the example your device is looking for - ESP32 only")
#endif

// ##### load the ESP32 preferences facilites
#include <Preferences.h>
Preferences store;

// LoRaWAN config, credentials & pinmap
#include "config.h" 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RadioLib.h>
#include <waveAction_inferencing.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
int box[3] = {0};
int run = 0;
int f = 0;
/* Constant defines -------------------------------------------------------- */
MPU6050 imu;
int16_t ax, ay, az;
// GPIO where the DS18B20 is connected to
const int oneWireBus = D0;     
#define ACC_RANGE           1 // 0: -/+2G; 1: +/-4G
#define CONVERT_G_TO_MS2    (9.81/(16384/(1.+ACC_RANGE)))
#define MAX_ACCEPTED_RANGE  (2*9.81)+(2*9.81)*ACC_RANGE   
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// utilities & vars to support ESP32 deep-sleep. The RTC_DATA_ATTR attribute
// puts these in to the RTC memory which is preserved during deep-sleep
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR uint16_t bootCountSinceUnsuccessfulJoin = 0;
RTC_DATA_ATTR uint8_t LWsession[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];

// abbreviated version from the Arduino-ESP32 package, see
// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/deepsleep.html
// for the complete set of options
 float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println(F("Wake from sleep"));
  } else {
    Serial.print(F("Wake not caused by deep sleep: "));
    Serial.println(wakeup_reason);
  }

  Serial.print(F("Boot count: "));
  Serial.println(++bootCount);      // increment before printing
}

// put device in to lowest power deep-sleep mode
void gotoSleep(uint32_t seconds) {
  esp_sleep_enable_timer_wakeup(seconds * 1000UL * 1000UL); // function uses uS
  Serial.println(F("Sleeping\n"));
  //Serial.flush();

  esp_deep_sleep_start();

  // if this appears in the serial debug, we didn't go to sleep!
  // so take defensive action so we don't continually uplink
  Serial.println(F("\n\n### Sleep failed, delay of 5 minutes & then restart ###\n"));
  delay(5UL * 60UL * 1000UL);
  ESP.restart();
}

int16_t lwActivate() {
  int16_t state = RADIOLIB_ERR_UNKNOWN;

  // setup the OTAA session information
  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);

  Serial.println(F("Recalling LoRaWAN nonces & session"));
  // ##### setup the flash storage
  store.begin("radiolib");
  // ##### if we have previously saved nonces, restore them and try to restore session as well
  if (store.isKey("nonces")) {
    uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];										// create somewhere to store nonces
    store.getBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);	// get them from the store
    state = node.setBufferNonces(buffer); 															// send them to LoRaWAN
    debug(state != RADIOLIB_ERR_NONE, F("Restoring nonces buffer failed"), state, false);

    // recall session from RTC deep-sleep preserved variable
    state = node.setBufferSession(LWsession); // send them to LoRaWAN stack

    // if we have booted more than once we should have a session to restore, so report any failure
    // otherwise no point saying there's been a failure when it was bound to fail with an empty LWsession var.
    debug((state != RADIOLIB_ERR_NONE) && (bootCount > 1), F("Restoring session buffer failed"), state, false);

    // if Nonces and Session restored successfully, activation is just a formality
    // moreover, Nonces didn't change so no need to re-save them
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("Succesfully restored session - now activating"));
      state = node.activateOTAA();
      debug((state != RADIOLIB_LORAWAN_SESSION_RESTORED), F("Failed to activate restored session"), state, true);

      // ##### close the store before returning
      store.end();
      return(state);
    }

  } else {  // store has no key "nonces"
    Serial.println(F("No Nonces saved - starting fresh."));
  }

  // if we got here, there was no session to restore, so start trying to join
  state = RADIOLIB_ERR_NETWORK_NOT_JOINED;
  while (state != RADIOLIB_LORAWAN_NEW_SESSION) {
    Serial.println(F("Join ('login') to the LoRaWAN Network"));
    state = node.activateOTAA();

    // ##### save the join counters (nonces) to permanent store
    Serial.println(F("Saving nonces to flash"));
    uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];           // create somewhere to store nonces
    uint8_t *persist = node.getBufferNonces();                  // get pointer to nonces
    memcpy(buffer, persist, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);  // copy in to buffer
    store.putBytes("nonces", buffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE); // send them to the store
    
    // we'll save the session after an uplink

    if (state != RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.print(F("Join failed: "));
      Serial.println(state);

      // how long to wait before join attempts. This is an interim solution pending 
      // implementation of TS001 LoRaWAN Specification section #7 - this doc applies to v1.0.4 & v1.1
      // it sleeps for longer & longer durations to give time for any gateway issues to resolve
      // or whatever is interfering with the device <-> gateway airwaves.
      uint32_t sleepForSeconds = min((bootCountSinceUnsuccessfulJoin++ + 1UL) * 60UL, 3UL * 60UL);
      Serial.print(F("Boots since unsuccessful join: "));
      Serial.println(bootCountSinceUnsuccessfulJoin);
      Serial.print(F("Retrying join in "));
      Serial.print(sleepForSeconds);
      Serial.println(F(" seconds"));
delay(30000);
      gotoSleep(sleepForSeconds);

    } // if activateOTAA state
  } // while join

  Serial.println(F("Joined"));

  // reset the failed join count
  bootCountSinceUnsuccessfulJoin = 0;

  delay(1000);  // hold off off hitting the airwaves again too soon - an issue in the US
  
  // ##### close the store
  store.end();  
  return(state);
}
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
* @brief      Arduino setup function
*/
// setup & execute all device functions ...
void setup() {
  Serial.begin(115200);
  //while (!Serial); 							// wait for serial to be initalised

  delay(2000);  									// give time to switch to the serial monitor
   Wire.begin();
   

  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);
  Serial.println(F("\nSetup"));
   pinMode(D2, OUTPUT);
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
  digitalWrite(D2, HIGH);
  delay(500);
  print_wakeup_reason();
  sensors.begin();
   imu.initialize();
    delay(10);
    imu.setXAccelOffset(-3940);
    imu.setYAccelOffset(1172);
    imu.setZAccelOffset(1114);
    imu.setXGyroOffset(-9);
    imu.setYGyroOffset(-40);
    imu.setZGyroOffset(-15);

    imu.setFullScaleAccelRange(ACC_RANGE);
  int16_t state = 0;  						// return value for calls to RadioLib

  // setup the radio based on the pinmap (connections) in config.h
  Serial.println(F("Initalise the radio"));
  state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initalise radio failed"), state, true);

  // activate node by restoring session or otherwise joining the network
  state = lwActivate();
  // state is one of RADIOLIB_LORAWAN_NEW_SESSION or RADIOLIB_LORAWAN_SESSION_RESTORED

  // ----- and now for the main event -----
  Serial.println(F("Sending uplink"));

  // this is the place to gather the sensor inputs
  // instead of reading any real sensor, we just generate some random numbers as example
  uint8_t value1 = radio.random(100);
  
  int analogValue = analogRead(D1);
  uint16_t value2 = analogValue;
  
  // print out the values you read:
  Serial.printf("ADC analog value = %d\n",analogValue);
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  delay(5000);
  waver();
  // build payload byte array
  uint8_t uplinkPayload[4];
  uplinkPayload[0] = temperatureF;
  uplinkPayload[1] = highByte(value2);   // See notes for high/lowByte functions
  uplinkPayload[2] = lowByte(value2);
  uplinkPayload[3] = f;
  
  // perform an uplink
  state = node.sendReceive(uplinkPayload, sizeof(uplinkPayload));    
  debug((state < RADIOLIB_ERR_NONE) && (state != RADIOLIB_ERR_NONE), F("Error in sendReceive"), state, false);

  Serial.print(F("FCntUp: "));
  Serial.println(node.getFCntUp());

  // now save session to RTC memory
  uint8_t *persist = node.getBufferSession();
  memcpy(LWsession, persist, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
  delay(20000);
  digitalWrite(D2,LOW);
  digitalWrite(D3,LOW);
  // wait until next uplink - observing legal & TTN FUP constraints
  gotoSleep(uplinkIntervalSeconds);

}


// The ESP32 wakes from deep-sleep and starts from the very beginning.
// It then goes back to sleep, so loop() is never called and which is
// why it is empty.

void loop() {}
void waver(){
 
  while(run < 11){
   run = run + 1;
    ei_printf("\nStarting inferencing in 2 seconds...\n");

    delay(2000);

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 1) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        imu.getAcceleration(&ax, &ay, &az);       
        //buffer[ix + 0] = ax;
        //buffer[ix + 1] = ay;
        buffer[ix ] = az;
       
        //ei_printf("raw values:    %.2f, %.2f, %.2f\n", ax*CONVERT_G_TO_MS2, ay*CONVERT_G_TO_MS2, az*CONVERT_G_TO_MS2);
        
        //buffer[ix + 0] *= CONVERT_G_TO_MS2;
        //buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix ] *= CONVERT_G_TO_MS2;

        //for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix ]) > MAX_ACCEPTED_RANGE) {
                buffer[ix] = ei_get_sign(buffer[ix]) * MAX_ACCEPTED_RANGE;
            }
        //}

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        box[ix] = box[ix] + result.classification[ix].value * 100;
    }
    if (run == 10) {
      int link = 0;
       int a = box[link]/10;
       box[link] = 0;
       int b = box[link +1]/10;
       box[link +1] = 0;
       int c = box[link +2]/10;
       box[link +2] = 0;

       int d = a + b;
       int e = b + c;
       Serial.println(d);
       Serial.println(e);
       f = abs(((float(d-e)/100) * 5) -5);
       Serial.print("Magic Number = ");
       Serial.println(f); 
       //run = 0;
    }
    }
}