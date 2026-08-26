// pedal-haptics — pin adapter. The logic lives in src/haptics_core.h.
// See spec §3.2 (switching scheme) and §3.5 (pinout).
#include "src/haptics_core.h"

#define FW_VERSION "0.1.0"

// Pinout, spec §3.5. Channel 0 = brake, channel 1 = throttle.
static const int kIn1[2] = {0, 2};   // AIN1, BIN1
static const int kIn2[2] = {1, 3};   // AIN2, BIN2
static const int kSleepPin = 4;      // nSLEEP
static const int kLedPin   = LED_BUILTIN;

static const uint32_t kPwmFreq  = 25000;  // ≥25 kHz, inaudible. Spec §3.4
static const uint8_t  kPwmRange = 255;

static HapticsCore core;

// Applies an output to the H-bridge.
//
// Slow decay scheme: IN1 fixed HIGH and PWM inverted on IN2.
//   IN2 LOW  → (1,0) → motor driven
//   IN2 HIGH → (1,1) → motor shorted (brake)
// With this inversion, duty 0 leaves the motor braked with no extra logic,
// which is exactly what we want for crisp pulses.
static void applyChannel(int ch, ChannelOut out) {
    if (out.coast) {
        digitalWrite(kIn1[ch], LOW);
        digitalWrite(kIn2[ch], LOW);
        return;
    }
    digitalWrite(kIn1[ch], HIGH);
    analogWrite(kIn2[ch], kPwmRange - out.duty);
}

void setup() {
    for (int ch = 0; ch < 2; ++ch) {
        pinMode(kIn1[ch], OUTPUT);
        pinMode(kIn2[ch], OUTPUT);
        digitalWrite(kIn1[ch], LOW);
        digitalWrite(kIn2[ch], LOW);
    }
    pinMode(kSleepPin, OUTPUT);
    digitalWrite(kSleepPin, HIGH);  // wake up the DRV8833
    pinMode(kLedPin, OUTPUT);

    analogWriteFreq(kPwmFreq);
    analogWriteRange(kPwmRange);

    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.printf("PH1 %s %u\n", FW_VERSION, (unsigned)kDutyCap);
}

void loop() {
    const uint32_t now = millis();

    while (Serial.available() > 0) {
        core.feed((uint8_t)Serial.read(), now);
    }
    core.tick(now);

    for (int ch = 0; ch < 2; ++ch) {
        applyChannel(ch, core.output(ch));
    }

    // The onboard LED mirrors channel 0's duty, without inversion: a permanent
    // status indicator and the only visual check possible without motors.
    analogWrite(kLedPin, core.output(0).duty);
}
