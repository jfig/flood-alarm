#include <RBD_Timer.h>
#include <RBD_Button.h>
#include <SimpleDHT.h>
#include <elapsedMillis.h>

const boolean TRUE = 1;
const boolean FALSE = 0;

// Heartbeat
elapsedMillis emHeartbeat;
const int HEARTBEAT_INTERVAL = 3000;
const int HEARTBEAT_DURATION = 30;
const int HEARTBEAT_PIN      = 13;

// DHT Sensor
SimpleDHT11 dht11;
elapsedMillis emDHT;
const int DTH_SENSOR_PIN = 12;
const int DTH_SENSOR_READ_FREQUENCY = 2000;
boolean gDTH_ERROR = FALSE;
byte    gTemperature = 0;
byte    gHumidity = 0;

//
const int FLOOD_SENSOR_PIN = 9;

// Alarm Options
boolean gAlarmInAction = FALSE;
boolean gMute = FALSE;
const int BUZZER_PIN = 6;
const int RED_LED_PIN = 2;

//const int MUTE_BUTTON_PIN = 8;
RBD::Button button(8);

// FLoodAlarm
const int FLOOD_ALARM_INTERVAL = 2000;
const int FLOOD_ALARM_PITCH = 2000;
const int FLOOD_ALARM_DURATON = 300;
elapsedMillis emFloodAlarm;
boolean gFloodAlarm = FALSE; // if True a water has been detected

// Humidity Alarm
const byte HUMIDITY_ALARM_LEVEL = 60;
const int  HUMIDITY_ALARM_INTERVAL = 5000;
const int  HUMIDITY_ALARM_PITCH = 5000;
const int  HUMIDITY_ALARM_DURATION = 100;
elapsedMillis emHumidityAlarm;

void setup() {
  Serial.begin(115200);

  pinMode(HEARTBEAT_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(FLOOD_SENSOR_PIN, INPUT);

  myTone(1000, 100);
  redLed(TRUE);
  delay(100);
  redLed(FALSE);

  // the first read of the buttun causes a false positive, so we're reading
  // it here on setup() to avoid causeng an error
  button.onReleased();

}

void loop() {
  checkFloodSensors();
  checkDHTsensor();
  checkMuteButton();

  floodAlarm();
  humidityAlarm();

  heartbeat();
}

void checkMuteButton() {
  if(button.onReleased()) {
    Serial.println("Button Released");
    gMute = !gMute;
    flipHeartbeatLED();
  }
}

/**
 *  Check the Flood Sensors, start alarm if water found
 */
void checkFloodSensors() {
  gFloodAlarm = digitalRead(FLOOD_SENSOR_PIN);
}

/**
 * floodAlarm Warnings
 */
void floodAlarm() {
  static boolean floodAlarmOngoing;

  // Check if another alarm other than this is ongoind and exit if so
  if(gAlarmInAction && !floodAlarmOngoing) return;

  if (gFloodAlarm && ( emFloodAlarm > FLOOD_ALARM_INTERVAL ) ) {
    myTone(FLOOD_ALARM_PITCH,FLOOD_ALARM_DURATON);
    redLed(TRUE);
    floodAlarmOngoing = TRUE;
    gAlarmInAction = TRUE;
    emFloodAlarm = 0;
    return;
  }

  if ( floodAlarmOngoing && ( emFloodAlarm > FLOOD_ALARM_DURATON ) ) {
    redLed(FALSE);
    floodAlarmOngoing = FALSE;
    gAlarmInAction = FALSE;
    emFloodAlarm = 0;
    return;
  }
}

/**
 * excessive Humidity Warnings
 */
void humidityAlarm() {
  static boolean humidityAlarmOngoing;

  // Check if another alarm other than this is ongoind and exit if so
  if(gAlarmInAction && !humidityAlarmOngoing) return;

  if ( ( gHumidity > HUMIDITY_ALARM_LEVEL ) &&
      ( emHumidityAlarm > HUMIDITY_ALARM_INTERVAL ) ) {

    myTone(HUMIDITY_ALARM_PITCH,HUMIDITY_ALARM_DURATION);
    redLed(TRUE);
    humidityAlarmOngoing = TRUE;
    gAlarmInAction = TRUE;
    emHumidityAlarm = 0;
    return;
  }

  if ( humidityAlarmOngoing && ( emHumidityAlarm > HUMIDITY_ALARM_DURATION ) ) {
    redLed(FALSE);
    humidityAlarmOngoing = FALSE;
    gAlarmInAction = FALSE;
    emHumidityAlarm = 0;
    return;
  }
}

/**
 * Reads Humidity and temperature from the DHT Sensor and updates the
 * corresponding global variables
 */
void checkDHTsensor() {
  if ( emDHT < DTH_SENSOR_READ_FREQUENCY) return;

  emDHT = 0;
  if (dht11.read(DTH_SENSOR_PIN, &gTemperature, &gHumidity, NULL)) {
    gDTH_ERROR = TRUE;
    redLed(TRUE);
    return;
  }
  gDTH_ERROR = FALSE;
  Serial.print((int)gTemperature); Serial.print(" *C, ");
  Serial.print((int)gHumidity); Serial.println(" %");
}

/**
 * Just controls the On/Off state of the Red LED
 */
void redLed(boolean OnOff) {
  digitalWrite(RED_LED_PIN, OnOff);
}


/**
 *  Blinks a LED giving visual indication that the software is runningl
 */
void heartbeat() {
  static boolean       hbStatus;           // status of heartbeat led on/off
  static int           hbTimeSinceStatus;  // time since status change
  if(!hbStatus) {
    // LED of OFF
    if ( emHeartbeat >= HEARTBEAT_INTERVAL ) {
      hbStatus = !hbStatus;
      flipHeartbeatLED();
      emHeartbeat = 0;
    }
  } else {
    //LED is ON
    if ( emHeartbeat >= HEARTBEAT_DURATION ) {
      hbStatus = !hbStatus;
      flipHeartbeatLED();
      emHeartbeat = 0;
    }
  }
}

void flipHeartbeatLED() {
  digitalWrite(HEARTBEAT_PIN, !digitalRead(HEARTBEAT_PIN));
}

void myTone(int frequency, int duration ) {
  if (!gMute) tone(BUZZER_PIN,frequency,duration);
}
