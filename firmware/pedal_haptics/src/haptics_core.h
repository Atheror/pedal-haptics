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
    bool    coast;  // true = high impedance; false = active brake if duty==0
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
        out.coast = tripped_;
        if (tripped_) {
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
            // Kick only when starting from rest, not on every duty change.
            if (duty[ch] > 0 && !running_[ch]) {
                kicking_[ch]       = true;
                kick_start_ms_[ch] = now_ms;
            }
            if (duty[ch] == 0) {
                kicking_[ch] = false;
            }
            requested_[ch] = duty[ch];
            running_[ch]   = duty[ch] > 0;
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
};
