// Pure logic for the pedal-haptics firmware: frame parsing, duty cap,
// watchdog, and overdrive kick. No Arduino dependencies, so it can be tested
// on the host. The .ino is just a pin adapter. See spec §4.
#pragma once

#include <stdint.h>

static const uint8_t  kPreamble    = 0xA5;
static const int      kFrameSize   = 5;
static const uint8_t  kDutyCap     = 179;  // 70% of 255, see spec §4.3
static const uint32_t kWatchdogMs  = 250;  // spec §4.2
static const uint32_t kOverdriveMs = 20;   // spec §3.2
// Minimum time a channel must sit at duty 0 before another overdrive kick is
// armed. The kick is the one sanctioned exception to the duty cap (spec §4.3),
// so re-arming it on every 0->nonzero edge would hand a pulse train a
// permanent 255: at --pulse 25 the 20 ms kick covers the whole on-phase and
// the cap is never applied at all. It also keeps the DRV8833 in a fresh inrush
// on every cycle. A kick only helps a mass that has actually stopped.
static const uint32_t kMinRestMs = 50;

static const uint8_t kFlagBrakeCh0 = 1 << 0;
static const uint8_t kFlagBrakeCh1 = 1 << 1;

// Out-of-band request for the identification banner, spec §4.1: the firmware
// answers "solo al conectar y ante 0x3F". The RP2040 does not reset when the
// host opens its CDC port -- unlike an AVR board, where DTR toggles a reset
// line -- so setup()'s banner is printed once per power cycle and every later
// connection would find the board mute without this query.
static const uint8_t kQueryBanner = 0x3F;

struct ChannelOut {
    uint8_t duty;   // 0..255, already capped
    // true = high impedance; false = active brake if duty == 0.
    //
    // Nothing sets this today: the watchdog brakes (see output()) and duty 0
    // brakes too. It stays because spec §3.2's stop sequence ends in coast,
    // and whatever adds that timed hand-off will need this to be an explicit,
    // deliberate request rather than the watchdog's silent default.
    bool    coast;
};

class HapticsCore {
public:
    HapticsCore() { reset(); }

    // Feeds a byte received over serial.
    void feed(uint8_t b, uint32_t now_ms) {
        if (buf_len_ == 0) {
            // Between frames only. Mid-frame a 0x3F byte is a perfectly
            // ordinary duty, flags or checksum value and must stay frame data,
            // which is why this is gated on buf_len_ == 0 rather than checked
            // for every byte.
            if (b == kQueryBanner) {
                wants_banner_ = true;
                return;  // never enters the frame buffer
            }
            if (b != kPreamble) {
                return;  // waiting for preamble: discard noise
            }
        }
        buf_[buf_len_++] = b;
        if (buf_len_ < kFrameSize) {
            return;
        }
        buf_len_ = 0;

        const uint8_t sum = buf_[0] ^ buf_[1] ^ buf_[2] ^ buf_[3];
        if (sum != buf_[4]) {
            ++rejected_;
            return;
        }
        ++accepted_;
        applyFrame(buf_[1], buf_[2], buf_[3], now_ms);
    }

    // Advances time: watchdog and end of overdrive.
    void tick(uint32_t now_ms) {
        if (now_ms - last_frame_ms_ > kWatchdogMs) {
            tripped_ = true;
            for (int ch = 0; ch < 2; ++ch) {
                requested_[ch] = 0;
                running_[ch]   = false;
                kicking_[ch]   = false;
            }
            // rest_since_ms_ is deliberately not stamped here. A trip means
            // kWatchdogMs (250 ms) passed with no frame, which already
            // exceeds kMinRestMs, so the first frame after a trip is entitled
            // to its kick; and stamping would refresh on every tick while
            // tripped, so no channel would ever qualify again.
        }
        // Unsigned subtraction, same pattern as the watchdog above: it stays
        // correct across a millis() rollover. An absolute `now_ms >= deadline`
        // comparison would not -- near UINT32_MAX the deadline itself wraps to
        // a small value and the kick ends immediately.
        for (int ch = 0; ch < 2; ++ch) {
            if (kicking_[ch] && now_ms - kick_start_ms_[ch] >= kOverdriveMs) {
                kicking_[ch] = false;
            }
        }
    }

    ChannelOut output(int ch) const {
        ChannelOut out;
        out.coast = false;
        if (tripped_) {
            // Watchdog: active brake (duty 0, no coast), not a coast.
            //
            // Spec §4.2 brakes both channels; coasting alone lets the
            // eccentric mass spin down for ~80 ms against ~10-15 ms braked
            // (spec §3.2). Braking also keeps the .ino on its ordinary
            // digitalWrite(IN1, HIGH) + analogWrite(IN2, ...) path: the coast
            // branch is the only place a digitalWrite lands on a pad that
            // analogWrite currently owns with no pinMode in between, and if
            // the pad is still in GPIO_FUNC_PWM that write may never reach
            // the pin -- leaving IN1 LOW with IN2 still PWM'd, which the
            // bridge reads as drive in reverse for as long as the board has
            // power. Exactly what the watchdog exists to prevent. The CLI
            // deliberately never sends a stop frame, so this runs on every
            // single invocation.
            //
            // Duty 0 unbraked is already the resting state, so this adds no
            // new steady state, only removes a hazardous one.
            out.duty = 0;
            return out;
        }
        if (kicking_[ch]) {
            out.duty = 255;  // the kick is the only exception to the cap
            return out;
        }
        out.duty = requested_[ch] > kDutyCap ? kDutyCap : requested_[ch];
        return out;
    }

    uint32_t framesAccepted() const { return accepted_; }
    uint32_t framesRejected() const { return rejected_; }
    bool     watchdogTripped() const { return tripped_; }

    // True when a 0x3F query arrived and the banner has not been reprinted
    // yet. The .ino prints it and calls clearBanner().
    bool wantsBanner() const { return wants_banner_; }
    void clearBanner() { wants_banner_ = false; }

private:
    void reset() {
        buf_len_ = 0;
        accepted_ = 0;
        rejected_ = 0;
        tripped_ = false;
        wants_banner_ = false;
        last_frame_ms_ = 0;
        for (int ch = 0; ch < 2; ++ch) {
            requested_[ch] = 0;
            running_[ch] = false;
            kicking_[ch] = false;
            kick_start_ms_[ch] = 0;
            rest_since_ms_[ch] = 0;
            ever_ran_[ch] = false;
        }
    }

    void applyFrame(uint8_t d0, uint8_t d1, uint8_t flags, uint32_t now_ms) {
        last_frame_ms_ = now_ms;
        tripped_ = false;

        const uint8_t duty[2] = {
            (flags & kFlagBrakeCh0) ? uint8_t(0) : d0,
            (flags & kFlagBrakeCh1) ? uint8_t(0) : d1,
        };

        for (int ch = 0; ch < 2; ++ch) {
            // Kick only when starting from rest, not on every duty change --
            // and only when the channel has actually rested long enough for
            // the mass to stop. Unsigned subtraction, same pattern as the
            // watchdog and the kick timeout: `now_ms >= since + kMinRestMs`
            // would break across a millis() rollover.
            if (duty[ch] > 0 && !running_[ch]) {
                const bool rested =
                    !ever_ran_[ch] || now_ms - rest_since_ms_[ch] >= kMinRestMs;
                if (rested) {
                    kicking_[ch]       = true;
                    kick_start_ms_[ch] = now_ms;
                }
            }
            if (duty[ch] == 0) {
                kicking_[ch] = false;
                // Stamp only the transition into rest. Refreshing on every
                // frame at 0 would push the deadline forever and no channel
                // would ever qualify.
                if (running_[ch]) {
                    rest_since_ms_[ch] = now_ms;
                }
            }
            requested_[ch] = duty[ch];
            running_[ch]   = duty[ch] > 0;
            if (duty[ch] > 0) {
                ever_ran_[ch] = true;
            }
        }
    }

    uint8_t  buf_[kFrameSize];
    int      buf_len_;
    uint32_t accepted_;
    uint32_t rejected_;
    bool     tripped_;
    bool     wants_banner_;
    uint32_t last_frame_ms_;
    uint8_t  requested_[2];
    bool     running_[2];
    bool     kicking_[2];
    uint32_t kick_start_ms_[2];
    uint32_t rest_since_ms_[2];  // when the channel last dropped to duty 0
    bool     ever_ran_[2];       // false = never ran, eligible to kick at once
};
