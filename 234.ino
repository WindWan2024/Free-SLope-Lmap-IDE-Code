/*
  Foldable LED Lamp - V7 BLE test
  Board: Seeed Studio XIAO ESP32-C3

  Touch:
    D2, active HIGH
    Single tap: ON / OFF
    Double tap: Toggle night mode
    Long press: Adjust brightness
    Long-press directions alternate: increase, decrease...

  LED:
    D10, 5000 Hz, 8-bit PWM

  BLE:
    Device name: FoldLamp-C3
    Nordic UART Service

    Commands written to RX:
      ON
      OFF
      NIGHT_ON
      NIGHT_OFF
      BRIGHTNESS=128

    Read/subscribe to TX for status updates.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <stdlib.h>

// ---------- Pins and PWM ----------
const int LED_PWM_PIN = D10;
const int TOUCH_PIN = D2;

const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

// ---------- Brightness ----------
const int MIN_BRIGHTNESS = 10;
const int MAX_BRIGHTNESS = 255;
const int START_BRIGHTNESS = 150;
const int NIGHT_BRIGHTNESS = 40;
const int BRIGHTNESS_STEP = 4;

// ---------- Touch timing ----------
const unsigned long DEBOUNCE_MS = 5;
const unsigned long DOUBLE_TAP_MS = 230;
const unsigned long LONG_PRESS_MS = 280;
const unsigned long RAMP_INTERVAL_MS = 10;
const uint32_t BLE_IDLE_TIMEOUT_MS = 15UL * 60UL * 1000UL;

// ---------- Nordic UART Service UUIDs ----------
#define SERVICE_UUID             "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// ---------- Lamp state ----------
bool ledOn = false;
bool nightMode = false;
int brightness = START_BRIGHTNESS;
int savedNormalBrightness = START_BRIGHTNESS;

// ---------- Touch debounce ----------
bool rawTouch = false;
bool stableTouch = false;
unsigned long rawChangedAt = 0;

// ---------- Current press ----------
unsigned long pressStartedAt = 0;
unsigned long lastRampAt = 0;
bool longPressActive = false;
bool boundaryLocked = false;

// ---------- Tap recognition ----------
bool waitingForSingleTap = false;
bool secondTapCandidate = false;
unsigned long firstTapReleasedAt = 0;

// ---------- Long-press direction ----------
bool nextLongPressIncreases = true;
bool currentLongPressIncreases = true;

// ---------- BLE state ----------
BLEServer* bleServer = nullptr;
BLECharacteristic* txCharacteristic = nullptr;

volatile bool bleConnected = false;
volatile bool advertisingRestartPending = false;
volatile unsigned long disconnectedAt = 0;
volatile uint32_t lastBleActivityAt = 0;
bool bleRadioEnabled = false;
bool previousBleConnected = false;

struct BleCommand {
  char text[40];
};

QueueHandle_t bleCommandQueue = nullptr;


// =====================================================
// Lamp output and status
// =====================================================

void updateLampOutput() {
  ledcWrite(LED_PWM_PIN, ledOn ? brightness : 0);
}

String currentStatus() {
  String status = "STATE=";
  status += ledOn ? "ON" : "OFF";
  status += ";BRIGHTNESS=";
  status += String(brightness);
  status += ";NIGHT=";
  status += nightMode ? "1" : "0";
  status += ";NEXT_LONG_PRESS=";
  status += nextLongPressIncreases ? "UP" : "DOWN";
  return status;
}

void sendBleStatus(const String& message) {
  Serial.print("BLE TX: ");
  Serial.println(message);

  if (txCharacteristic != nullptr) {
    txCharacteristic->setValue(message);

    if (bleConnected) {
      txCharacteristic->notify();
    }
  }
}

void reportState() {
  sendBleStatus(currentStatus());
}


// =====================================================
// Power control
// =====================================================

void turnLampOn() {
  ledOn = true;
  nightMode = false;

  // Every OFF -> ON transition starts at brightness 180.
  brightness = START_BRIGHTNESS;
  savedNormalBrightness = START_BRIGHTNESS;
  nextLongPressIncreases = true;

  updateLampOutput();
  Serial.println("Lamp ON: brightness 180");
}

void turnLampOff() {
  ledOn = false;
  nightMode = false;

  // Reset brightness and the next long-press direction.
  brightness = START_BRIGHTNESS;
  savedNormalBrightness = START_BRIGHTNESS;
  nextLongPressIncreases = true;

  updateLampOutput();
  Serial.println("Lamp OFF: next ON starts at 180");
}

void toggleLampPower() {
  if (ledOn) {
    turnLampOff();
  } else {
    turnLampOn();
  }

  if (!bleRadioEnabled) {
    startBle();
  }

  reportState();
}


// =====================================================
// Night mode
// =====================================================

void setNightModeOn() {
  if (nightMode) {
    sendBleStatus("OK:NIGHT_ALREADY_ON");
    return;
  }

  if (ledOn) {
    savedNormalBrightness = brightness;
  } else {
    // Night mode can turn the lamp on from OFF.
    savedNormalBrightness = START_BRIGHTNESS;
    nextLongPressIncreases = true;
  }

  ledOn = true;
  nightMode = true;
  brightness = NIGHT_BRIGHTNESS;

  updateLampOutput();
  Serial.println("Night mode ON: brightness 40");
  reportState();
}

void setNightModeOff() {
  if (!nightMode) {
    sendBleStatus("OK:NIGHT_ALREADY_OFF");
    return;
  }

  nightMode = false;
  ledOn = true;
  brightness = constrain(
    savedNormalBrightness,
    MIN_BRIGHTNESS,
    MAX_BRIGHTNESS
  );

  updateLampOutput();
  Serial.println("Night mode OFF");
  reportState();
}

void toggleNightMode() {
  if (nightMode) {
    setNightModeOff();
  } else {
    setNightModeOn();
  }

  if (!bleRadioEnabled) {
    startBle();
  }
}


// =====================================================
// BLE command handling
// =====================================================

bool parseBrightnessCommand(const String& command, int& valueOut) {
  const String prefix = "BRIGHTNESS=";

  if (!command.startsWith(prefix)) {
    return false;
  }

  String numberText = command.substring(prefix.length());
  numberText.trim();

  if (numberText.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < numberText.length(); i++) {
    if (!isDigit(numberText[i])) {
      return false;
    }
  }

  valueOut = numberText.toInt();
  return true;
}

void processBleCommand(const char* rawCommand) {
  String command(rawCommand);
  command.trim();
  command.toUpperCase();

  Serial.print("BLE RX: ");
  Serial.println(command);

  if (command == "ON") {
    if (!ledOn) {
      turnLampOn();
    } else if (nightMode) {
      // ON means normal mode if the lamp is already in night mode.
      setNightModeOff();
      return;
    }

    sendBleStatus("OK:ON");
    reportState();
    return;
  }

  if (command == "OFF") {
    turnLampOff();
    sendBleStatus("OK:OFF");
    reportState();
    return;
  }

  if (command == "NIGHT_ON") {
    setNightModeOn();
    return;
  }

  if (command == "NIGHT_OFF") {
    setNightModeOff();
    return;
  }

  int requestedBrightness = 0;

  if (parseBrightnessCommand(command, requestedBrightness)) {
    if (requestedBrightness < MIN_BRIGHTNESS ||
        requestedBrightness > MAX_BRIGHTNESS) {
      sendBleStatus("ERR:BRIGHTNESS_RANGE_10_TO_255");
      return;
    }

    if (!ledOn) {
      sendBleStatus("ERR:TURN_ON_FIRST");
      return;
    }

    // Manual brightness control exits night mode.
    nightMode = false;
    brightness = requestedBrightness;
    savedNormalBrightness = brightness;

    updateLampOutput();

    sendBleStatus("OK:BRIGHTNESS");
    reportState();
    return;
  }

  sendBleStatus("ERR:UNKNOWN_COMMAND");
}


// =====================================================
// BLE callbacks
// =====================================================

class LampServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    (void)server;
    bleConnected = true;
    lastBleActivityAt = millis();
    Serial.println("BLE: client connected");
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    bleConnected = false;
    advertisingRestartPending = true;
    disconnectedAt = millis();
    Serial.println("BLE: client disconnected");
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    lastBleActivityAt = millis();

    // Arduino-ESP32 BLE getValue() returns Arduino String.
    String value = characteristic->getValue();

    if (value.length() == 0 || bleCommandQueue == nullptr) {
      return;
    }

    BleCommand command = {};

    size_t copyLength = value.length();
    if (copyLength >= sizeof(command.text)) {
      copyLength = sizeof(command.text) - 1;
    }

    memcpy(command.text, value.c_str(), copyLength);
    command.text[copyLength] = '\0';

    if (xQueueSend(bleCommandQueue, &command, 0) != pdTRUE) {
      Serial.println("BLE: command queue full");
    }
  }
};

// Keep callback objects alive across BLE deinit/init cycles.
LampServerCallbacks lampServerCallbacks;
RxCallbacks rxCallbacks;

void startBle() {
  if (bleRadioEnabled) {
    return;
  }

  if (bleCommandQueue == nullptr) {
    bleCommandQueue = xQueueCreate(4, sizeof(BleCommand));
  }

  if (!BLEDevice::init("FoldLamp-C3")) {
    Serial.println("BLE: initialization failed");
    return;
  }

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(&lampServerCallbacks);

  BLEService* service = bleServer->createService(SERVICE_UUID);

  txCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  txCharacteristic->addDescriptor(new BLE2902());
  txCharacteristic->setValue("READY");

  BLECharacteristic* rxCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  rxCharacteristic->setCallbacks(&rxCallbacks);

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  bleRadioEnabled = true;
  bleConnected = false;
  advertisingRestartPending = false;
  previousBleConnected = false;
  lastBleActivityAt = millis();

  Serial.println("BLE: advertising as FoldLamp-C3");
}

void stopBleAfterIdle() {
  if (!bleRadioEnabled) {
    return;
  }

  Serial.println("BLE: 15 minutes idle; disconnecting and shutting down");

  if (bleServer != nullptr) {
    const auto peers = BLEDevice::getPeerDevices(false);
    for (const auto& peer : peers) {
      bleServer->disconnect(peer.first);
    }
    delay(100);
  }

  // Keep BLE memory allocated so BLEDevice::init() can run again on touch.
  BLEDevice::deinit(false);

  bleServer = nullptr;
  txCharacteristic = nullptr;
  bleConnected = false;
  advertisingRestartPending = false;
  previousBleConnected = false;
  bleRadioEnabled = false;

  Serial.println("BLE: radio off; touch lamp to enable advertising again");
}


// =====================================================
// Long-press dimming
// =====================================================

void startLongPress(unsigned long now) {
  if (!ledOn) {
    turnLampOn();
  }

  currentLongPressIncreases = nextLongPressIncreases;
  nextLongPressIncreases = !nextLongPressIncreases;

  longPressActive = true;
  lastRampAt = now;

  waitingForSingleTap = false;
  secondTapCandidate = false;

  if (nightMode) {
    nightMode = false;
    savedNormalBrightness = brightness;
    Serial.println("Night mode exited by long press");
  }

  Serial.print("LONG PRESS: ");
  Serial.println(currentLongPressIncreases ? "INCREASE" : "DECREASE");

  if (!bleRadioEnabled) {
    startBle();
    // BLE initialization can take time; don't make the dimmer jump ahead.
    lastRampAt = millis();
  }

  reportState();
}

void lockAtBoundary() {
  brightness = constrain(
    brightness,
    MIN_BRIGHTNESS,
    MAX_BRIGHTNESS
  );

  updateLampOutput();
  longPressActive = false;
  boundaryLocked = true;

  Serial.print("Brightness held at ");
  Serial.println(brightness);
  reportState();

  // Reaching a boundary does not change the next direction.
}

void adjustBrightness() {
  int proposedBrightness = brightness;

  if (currentLongPressIncreases) {
    proposedBrightness += BRIGHTNESS_STEP;

    if (proposedBrightness >= MAX_BRIGHTNESS) {
      brightness = MAX_BRIGHTNESS;
      lockAtBoundary();
      return;
    }
  } else {
    proposedBrightness -= BRIGHTNESS_STEP;

    if (proposedBrightness <= MIN_BRIGHTNESS) {
      brightness = MIN_BRIGHTNESS;
      lockAtBoundary();
      return;
    }
  }

  brightness = proposedBrightness;
  savedNormalBrightness = brightness;
  updateLampOutput();
}


// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(TOUCH_PIN, INPUT);

  ledcAttach(LED_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(LED_PWM_PIN, 0);

  rawTouch = (digitalRead(TOUCH_PIN) == HIGH);
  stableTouch = rawTouch;
  rawChangedAt = millis();

  startBle();

  Serial.println("Foldable LED Lamp V7 BLE test ready");
  updateLampOutput();
}


// =====================================================
// Main loop
// =====================================================

void loop() {
  const unsigned long now = millis();

  // Process BLE writes outside the BLE callback.
  BleCommand receivedCommand;

  while (bleCommandQueue != nullptr &&
         xQueueReceive(bleCommandQueue, &receivedCommand, 0) == pdTRUE) {
    processBleCommand(receivedCommand.text);
  }

  // Restart advertising after a client disconnects, unless BLE was shut down.
  if (bleRadioEnabled &&
      advertisingRestartPending &&
      (now - disconnectedAt >= 200)) {
    advertisingRestartPending = false;
    BLEDevice::getAdvertising()->start();
    Serial.println("BLE: advertising restarted");
  }

  // After 15 quiet minutes, disconnect and fully deinitialize the BLE stack.
  // The processor, touch sensor polling, and lamp PWM remain active.
  if (bleRadioEnabled &&
      bleConnected &&
      (now - lastBleActivityAt >= BLE_IDLE_TIMEOUT_MS)) {
    stopBleAfterIdle();
  }

  // Report state when a client connects.
  if (bleConnected && !previousBleConnected) {
    reportState();
  }
  previousBleConnected = bleConnected;

  // Debounce the active-HIGH touch signal.
  const bool reading = (digitalRead(TOUCH_PIN) == HIGH);

  if (reading != rawTouch) {
    rawTouch = reading;
    rawChangedAt = now;
  }

  bool touchStarted = false;
  bool touchReleased = false;

  if ((now - rawChangedAt >= DEBOUNCE_MS) &&
      (stableTouch != rawTouch)) {
    stableTouch = rawTouch;

    if (stableTouch) {
      touchStarted = true;
      if (bleRadioEnabled) {
        lastBleActivityAt = now;
      }
    } else {
      touchReleased = true;
      if (bleRadioEnabled) {
        lastBleActivityAt = now;
      }
    }
  }

  // Resolve a single tap after the double-tap window.
  if (waitingForSingleTap &&
      !secondTapCandidate &&
      (now - firstTapReleasedAt >= DOUBLE_TAP_MS)) {
    waitingForSingleTap = false;
    toggleLampPower();
  }

  if (touchStarted) {
    pressStartedAt = now;
    longPressActive = false;
    boundaryLocked = false;

    Serial.println("Touch start");

    if (waitingForSingleTap &&
        (now - firstTapReleasedAt <= DOUBLE_TAP_MS)) {
      secondTapCandidate = true;
    } else {
      secondTapCandidate = false;
    }
  }

  // Recognize long press.
  if (stableTouch &&
      !longPressActive &&
      !boundaryLocked &&
      (now - pressStartedAt >= LONG_PRESS_MS)) {

    // The earlier short touch counts as a single tap if this
    // possible second tap is held long enough to become a long press.
    if (secondTapCandidate && waitingForSingleTap) {
      waitingForSingleTap = false;
      secondTapCandidate = false;
      toggleLampPower();
    }

    startLongPress(now);
  }

  // Ramp brightness while the finger remains down.
  if (stableTouch &&
      longPressActive &&
      (now - lastRampAt >= RAMP_INTERVAL_MS)) {
    lastRampAt = now;
    adjustBrightness();
  }

  if (touchReleased) {
    Serial.println("Touch release");

    if (boundaryLocked || longPressActive) {
      boundaryLocked = false;
      longPressActive = false;
      waitingForSingleTap = false;
      secondTapCandidate = false;
      Serial.println("Long press ended");
    } else if (secondTapCandidate) {
      secondTapCandidate = false;
      waitingForSingleTap = false;
      toggleNightMode();
    } else {
      waitingForSingleTap = true;
      firstTapReleasedAt = now;
    }
  }
}
