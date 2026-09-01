#include <SPI.h>
#include "w5500.h"

// W5500 pins
constexpr uint8_t W5500_MISO = 32;
constexpr uint8_t W5500_RST  = 26;
constexpr uint8_t W5500_MOSI = 23;
constexpr uint8_t W5500_SCK  = 18;
constexpr uint8_t W5500_CS   = 5;

// Four MOSFET-driver inputs
constexpr uint8_t DRIVER_CH1_PIN = 13;
constexpr uint8_t DRIVER_CH2_PIN = 33;
constexpr uint8_t DRIVER_CH_PIN = 14;
constexpr uint8_t DRIVER_CH4_PIN = 27;

// Timing for solenoids
constexpr unsigned long OPEN_MILLIS = 500;
constexpr uint8_t OPEN_PWM = 255;
constexpr uint8_t HOLD_PWM = 77;
constexpr uint32_t PWM_FREQUENCY_HZ = 200;  // Conservative value for the linked MOSFET module

// MAC addresses
static const uint8_t MAC_GROUND_STATION[6] = {0x02, 0x47, 0x53,
                                              0x00, 0x00, 0x01}; // GS: ground station
static const uint8_t MAC_RELIEF_VALVE[6] = {0x02, 0x52, 0x56,
                                            0x00, 0x00, 0x02}; // vent
static const uint8_t MAC_FLOW_VALVE[6] = {0x02, 0x46, 0x4C,
                                          0x00, 0x00, 0x03}; // valve
static const uint8_t MAC_SENSOR_GIGA[6] = {0x02, 0x53, 0x49,
                                           0x00, 0x00, 0x04}; // sensor

static const uint8_t LNG_PRES_MASK = 0b1000000; // LNG Pressurant / Upstream
static const uint8_t LOX_PRES_MASK = 0b0100000; // LOX Pressurant / Upstream
static const uint8_t GN2_VENT_MASK = 0b0010000; // GN2 Vent, currently removed
static const uint8_t LNG_FLOW_MASK = 0b0001000; // LNG Flow / Downstream
static const uint8_t LNG_VENT_MASK = 0b0000100; // LNG Vent
static const uint8_t LOX_FLOW_MASK = 0b0000010; // Lox Flow / Downstream
static const uint8_t LOX_VENT_MASK = 0b0000001; // LOX Vent

// NOTE:
// Your original sketch controlled seven outputs, but the pin list you supplied
// contains four MOSFET signal GPIOs. This build maps channels 1-4 to the first
// four logical outputs from the original sketch. Change only these four rows if
// your physical channel-to-valve assignment is different.
struct Valve {
  const char *name;
  uint8_t mask;
  uint8_t pin;
  bool state;
  unsigned long lastOpened;
};

Valve valves[] = {
    {"GN2_VENT",    GN2_VENT_MASK,   DRIVER_CH1_PIN, false, 0},
    {"LNG_VENT",    LNG_VENT_MASK,   DRIVER_CH2_PIN, false, 0},
    {"LOX_VENT",    LOX_VENT_MASK,   DRIVER_CH3_PIN, false, 0},
};

constexpr size_t NUM_VALVES = sizeof(valves) / sizeof(valves[0]);

uint8_t buffer[256];
uint8_t rocketState = 0;
bool ethernetReady = false;

// Explicit custom SPI routing: CS, SCK, MISO, MOSI.
Wiznet5500 w5500(W5500_CS, W5500_SCK, W5500_MISO, W5500_MOSI);

void setAllValvesOff() {
  for (size_t i = 0; i < NUM_VALVES; i++) {
    valves[i].state = false;
    analogWrite(valves[i].pin, 0);
  }
}

void resetW5500() {
  // W5500 RSTn is active low. Datasheet minimum low time is 500 us.
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, HIGH);
  delay(1);
  digitalWrite(W5500_RST, LOW);
  delay(1);
  digitalWrite(W5500_RST, HIGH);
  delay(2);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Put actuator outputs in their safe OFF state before bringing up Ethernet.
  for (size_t i = 0; i < NUM_VALVES; i++) {
    pinMode(valves[i].pin, OUTPUT);
    digitalWrite(valves[i].pin, LOW);
    analogWriteFrequency(valves[i].pin, PWM_FREQUENCY_HZ);
  }

  // Keep W5500 deselected during reset/startup.
  pinMode(W5500_CS, OUTPUT);
  digitalWrite(W5500_CS, HIGH);
  resetW5500();

  ethernetReady = w5500.begin(MAC_RELIEF_VALVE);

  if (!ethernetReady) {
    Serial.println("ERROR: W5500 failed to enter MACRAW mode. Outputs remain OFF.");
    setAllValvesOff();
    return;
  }

  Serial.println("ESP32 actuator initialized");
  Serial.println("W5500: MISO=32 RST=26 MOSI=23 SCK=18 CS=5");
  Serial.println("MOSFET inputs: CH1=13 CH2=12 CH3=14 CH4=27");
}

void receiveRocketState() {
  if (!ethernetReady) {
    return;
  }

  uint16_t len;
  while ((len = w5500.readFrame(buffer, sizeof(buffer))) > 0) {
    // Destination MAC (6) + source MAC (6) + EtherType (2) + state byte (1)
    if (len < 15) {
      continue;
    }

    // Custom EtherType 0x63E4 from the original project.
    if (buffer[12] != 0x63 || buffer[13] != 0xE4) {
      continue;
    }

    const uint8_t newState = buffer[14] & 0x7F;
    if (newState != rocketState) {
      rocketState = newState;
      Serial.println("----------");
      Serial.print("Received Rocket State: ");
      Serial.println(rocketState, BIN);
    }
  }
}

void updateValveStates() {
  for (size_t i = 0; i < NUM_VALVES; i++) {
    const bool shouldBeOpen = (rocketState & valves[i].mask) != 0;

    if (shouldBeOpen && !valves[i].state) {
      Serial.print(valves[i].name);
      Serial.println(": OPENING");
      valves[i].state = true;
      valves[i].lastOpened = millis();
    } else if (!shouldBeOpen && valves[i].state) {
      Serial.print(valves[i].name);
      Serial.println(": CLOSING");
      valves[i].state = false;
    }
  }
}

void applyValveVoltages() {
  for (size_t i = 0; i < NUM_VALVES; i++) {
    if (valves[i].state) {
      const unsigned long elapsed = millis() - valves[i].lastOpened;
      const uint8_t pwm = (elapsed > OPEN_MILLIS) ? HOLD_PWM : OPEN_PWM;
      analogWrite(valves[i].pin, pwm);
    } else {
      analogWrite(valves[i].pin, 0);
    }
  }
}

void loop() {
  if (!ethernetReady) {
    setAllValvesOff();
    delay(100);
    return;
  }

  receiveRocketState();
  updateValveStates();
  applyValveVoltages();
}
