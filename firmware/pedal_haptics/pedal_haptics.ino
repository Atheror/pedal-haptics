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
    // Coast is the one branch that digitalWrites a pad analogWrite owns with
    // no pinMode in between; if the pad is still in GPIO_FUNC_PWM the write
    // may not land. Nothing in the core requests it today -- the watchdog
    // brakes instead -- and whatever adds spec §3.2's brake-then-coast
    // hand-off must reclaim the pad with pinMode(OUTPUT) first.
    if (out.coast) {
        digitalWrite(kIn1[ch], LOW);
        digitalWrite(kIn2[ch], LOW);
        return;
    }
    digitalWrite(kIn1[ch], HIGH);
    analogWrite(kIn2[ch], kPwmRange - out.duty);
}

// Prints the identification line the host parses in its handshake, spec §4.1.
// Single definition on purpose: setup() and loop() must emit byte-identical
// banners, or a reconnect would parse differently from a cold boot.
static void printBanner() {
    Serial.printf("PH1 %s %u\n", FW_VERSION, (unsigned)kDutyCap);
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
    printBanner();
}

void loop() {
    const uint32_t now = millis();

    while (Serial.available() > 0) {
        core.feed((uint8_t)Serial.read(), now);
    }

    // Opening the CDC port does not reset an RP2040, so setup()'s banner is
    // long gone by the time a second host run connects. 0x3F asks for it
    // again; without this the handshake would only ever succeed once per
    // power cycle. Spec §4.1.
    if (core.wantsBanner()) {
        printBanner();
        core.clearBanner();
    }

    core.tick(now);

    for (int ch = 0; ch < 2; ++ch) {
        applyChannel(ch, core.output(ch));
    }

    // The onboard LED mirrors channel 0's duty, without inversion: a permanent
    // status indicator and the only visual check possible without motors.
    analogWrite(kLedPin, core.output(0).duty);
}
