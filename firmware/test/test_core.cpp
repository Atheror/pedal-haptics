// Host tests for haptics_core.h. No Arduino, no hardware.
#include "../pedal_haptics/src/haptics_core.h"
#include <cstdio>
#include <cstdlib>

static int g_failures = 0;

#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, msg);      \
            ++g_failures;                                                  \
        }                                                                  \
    } while (0)

// Feeds a complete, well-formed frame.
static void sendFrame(HapticsCore& c, uint8_t d0, uint8_t d1, uint8_t flags,
                      uint32_t now) {
    uint8_t b[kFrameSize];
    b[0] = kPreamble;
    b[1] = d0;
    b[2] = d1;
    b[3] = flags;
    b[4] = b[0] ^ b[1] ^ b[2] ^ b[3];
    for (int i = 0; i < kFrameSize; ++i) c.feed(b[i], now);
}

static void test_accepts_valid_frame() {
    HapticsCore c;
    sendFrame(c, 100, 50, 0, 1000);
    CHECK(c.framesAccepted() == 1, "should accept a valid frame");
    CHECK(c.framesRejected() == 0, "should not reject anything");
}

static void test_rejects_bad_checksum() {
    HapticsCore c;
    uint8_t b[kFrameSize] = {kPreamble, 10, 20, 0, 0xFF};
    for (int i = 0; i < kFrameSize; ++i) c.feed(b[i], 1000);
    CHECK(c.framesRejected() == 1, "should reject bad checksum");
    CHECK(c.framesAccepted() == 0, "should not accept the frame");
}

static void test_resyncs_after_garbage() {
    HapticsCore c;
    // Noise before a valid frame: the parser must resync.
    c.feed(0x00, 1000);
    c.feed(0xFF, 1000);
    c.feed(0x7E, 1000);
    sendFrame(c, 77, 0, 0, 1000);
    CHECK(c.framesAccepted() == 1, "should resync after garbage");
    c.tick(1000 + kOverdriveMs + 1); // let the kick pass, otherwise output() gives 255
    CHECK(c.output(0).duty == 77, "should apply the duty from the valid frame");
}

static void test_applies_duty_cap() {
    HapticsCore c;
    sendFrame(c, 255, 255, 0, 1000);
    c.tick(1000 + kOverdriveMs + 1); // let the overdrive pass
    CHECK(c.output(0).duty == kDutyCap, "should cap channel 0");
    CHECK(c.output(1).duty == kDutyCap, "should cap channel 1");
}

static void test_duty_below_cap_passes_through() {
    HapticsCore c;
    sendFrame(c, 50, 0, 0, 1000);
    c.tick(1000 + kOverdriveMs + 1);
    CHECK(c.output(0).duty == 50, "a duty below the cap is left untouched");
}

static void test_overdrive_kick_on_start_from_rest() {
    HapticsCore c;
    sendFrame(c, 100, 0, 0, 1000);
    // Immediately after starting from rest: full duty.
    CHECK(c.output(0).duty == 255, "should give the kick on start");
    c.tick(1000 + kOverdriveMs + 1);
    CHECK(c.output(0).duty == 100, "should drop to the requested duty after the kick");
}

static void test_no_kick_when_already_running() {
    HapticsCore c;
    sendFrame(c, 100, 0, 0, 1000);
    c.tick(1000 + kOverdriveMs + 1);
    sendFrame(c, 120, 0, 0, 1100);
    CHECK(c.output(0).duty == 120, "should not kick if it was already running");
}

static void test_watchdog_trips_after_timeout() {
    HapticsCore c;
    sendFrame(c, 150, 150, 0, 1000);
    c.tick(1000 + kWatchdogMs - 1);
    CHECK(!c.watchdogTripped(), "should not trip before the deadline");
    c.tick(1000 + kWatchdogMs + 1);
    CHECK(c.watchdogTripped(), "should trip past the deadline");
    CHECK(c.output(0).duty == 0, "channel 0 at zero");
    CHECK(c.output(1).duty == 0, "channel 1 at zero");
}

static void test_watchdog_recovers_on_new_frame() {
    HapticsCore c;
    sendFrame(c, 150, 0, 0, 1000);
    c.tick(1000 + kWatchdogMs + 1);
    CHECK(c.watchdogTripped(), "precondition: watchdog tripped");
    sendFrame(c, 90, 0, 0, 2000);
    c.tick(2000);
    CHECK(!c.watchdogTripped(), "should recover with a new frame");
}

static void test_zero_duty_brakes_not_coasts() {
    HapticsCore c;
    sendFrame(c, 0, 0, 0, 1000);
    // Duty 0 without watchdog = active brake, not coast. See spec §3.2 and §4.1.
    CHECK(!c.output(0).coast, "duty 0 should brake, not coast");
}

static void test_watchdog_ends_in_brake() {
    HapticsCore c;
    sendFrame(c, 150, 150, 0, 1000);
    c.tick(1000 + kWatchdogMs + 20);
    CHECK(c.watchdogTripped(), "precondition: the watchdog tripped");
    // Brake, not coast. Coasting leaves the mass spinning ~80 ms (spec §3.2),
    // and in the .ino the coast branch is the one that writes digitalWrite
    // over a pad analogWrite owns -- which can leave the bridge driving in
    // reverse indefinitely. Braking uses the same pin ops as normal running.
    for (int ch = 0; ch < 2; ++ch) {
        CHECK(!c.output(ch).coast, "the watchdog must brake, not coast");
        CHECK(c.output(ch).duty == 0, "the watchdog must hold duty at 0");
    }
}

static void test_brake_flag_forces_zero() {
    HapticsCore c;
    sendFrame(c, 200, 200, kFlagBrakeCh0, 1000);
    CHECK(c.output(0).duty == 0, "the brake flag should zero out channel 0");
    CHECK(c.output(1).duty > 0, "channel 1 should not be affected");
}

static void test_brake_flag_forces_zero_ch1() {
    HapticsCore c;
    sendFrame(c, 200, 200, kFlagBrakeCh1, 1000);
    CHECK(c.output(1).duty == 0, "the brake flag should zero channel 1");
    CHECK(c.output(0).duty > 0, "channel 0 should be unaffected");
}

static void test_query_byte_requests_banner() {
    HapticsCore c;
    CHECK(!c.wantsBanner(), "no banner is pending on a fresh core");
    c.feed(kQueryBanner, 1000);
    CHECK(c.wantsBanner(), "0x3F between frames should request the banner");
}

static void test_banner_flag_clears() {
    HapticsCore c;
    c.feed(kQueryBanner, 1000);
    c.clearBanner();
    CHECK(!c.wantsBanner(), "clearBanner() should consume the request");
    // And it must not re-arm itself, or the .ino would reprint on every loop().
    c.feed(0x00, 1010);
    CHECK(!c.wantsBanner(), "the request must not come back on its own");
}

static void test_query_byte_midframe_is_frame_data() {
    HapticsCore c;
    // 0x3F is a perfectly ordinary duty value. Inside a frame it must be
    // consumed as frame data, not swallowed as a banner query -- otherwise the
    // byte would vanish, the frame would desync, and every duty of 63 would
    // break the stream. This is why the check is gated on buf_len_ == 0.
    sendFrame(c, kQueryBanner, kQueryBanner, 0, 1000);
    CHECK(!c.wantsBanner(), "0x3F mid-frame must not request the banner");
    CHECK(c.framesAccepted() == 1, "the frame carrying 0x3F must still parse");
    c.tick(1000 + kOverdriveMs + 1);  // let the kick pass
    CHECK(c.output(0).duty == kQueryBanner, "duty 0x3F should reach channel 0");
    CHECK(c.output(1).duty == kQueryBanner, "duty 0x3F should reach channel 1");
}

static void test_query_byte_after_preamble_is_frame_data() {
    HapticsCore c;
    // Same point, one step earlier: only the preamble has arrived, so the
    // parser is mid-frame and this 0x3F is the ch0 duty byte.
    const uint8_t d0 = kQueryBanner, d1 = 0, flags = 0;
    c.feed(kPreamble, 1000);
    c.feed(d0, 1000);
    CHECK(!c.wantsBanner(), "0x3F right after the preamble is the ch0 duty");
    c.feed(d1, 1000);
    c.feed(flags, 1000);
    c.feed(uint8_t(kPreamble ^ d0 ^ d1 ^ flags), 1000);
    CHECK(c.framesAccepted() == 1, "the frame should complete normally");
    CHECK(!c.wantsBanner(), "still no banner request after the frame closes");
}

static void test_overdrive_kick_survives_millis_wraparound() {
    HapticsCore c;
    const uint32_t near_end = 0xFFFFFFFFu - 5;  // wraps 6 ms from here
    sendFrame(c, 100, 0, 0, near_end);
    CHECK(c.output(0).duty == 255, "the kick should start normally near the wrap");

    // Still BEFORE the wrap, so now_ms is numerically huge. This is the step
    // that kills the mutant: an absolute `now_ms >= deadline` check compares a
    // huge now_ms against an already-overflowed deadline (near_end + 20 wraps
    // to 14) and ends the kick here, 2 ms in instead of 20. Ticking only with
    // post-wrap values passes under both the broken and the fixed code.
    c.tick(near_end + 2);
    CHECK(c.output(0).duty == 255, "the kick must not end 2 ms in");

    c.tick(4);  // 10 ms elapsed, across the rollover
    CHECK(c.output(0).duty == 255, "the kick should survive the millis() wrap");

    c.tick(19);  // 25 ms elapsed, past kOverdriveMs
    CHECK(c.output(0).duty == 100, "the kick should end 20 ms after it started");
}

static void test_watchdog_survives_millis_wraparound() {
    HapticsCore c;
    const uint32_t near_end = 0xFFFFFFFFu - 100;
    sendFrame(c, 150, 0, 0, near_end);

    // Still before the wrap. A naive `now_ms > last_frame_ms_ + kWatchdogMs`
    // would compare a huge now_ms against an overflowed sum and trip here at
    // 50 ms. Without this step the test passes under that bug too.
    c.tick(near_end + 50);
    CHECK(!c.watchdogTripped(), "must not trip 50 ms in, before the wrap");

    c.tick(100);  // 201 ms elapsed, across the rollover
    CHECK(!c.watchdogTripped(), "should not trip before 250 ms across the wrap");

    c.tick(200);  // 301 ms elapsed
    CHECK(c.watchdogTripped(), "should trip after 250 ms across the wrap");
}

int main() {
    test_accepts_valid_frame();
    test_rejects_bad_checksum();
    test_resyncs_after_garbage();
    test_applies_duty_cap();
    test_duty_below_cap_passes_through();
    test_overdrive_kick_on_start_from_rest();
    test_no_kick_when_already_running();
    test_watchdog_trips_after_timeout();
    test_watchdog_recovers_on_new_frame();
    test_zero_duty_brakes_not_coasts();
    test_watchdog_ends_in_brake();
    test_brake_flag_forces_zero();
    test_brake_flag_forces_zero_ch1();
    test_query_byte_requests_banner();
    test_banner_flag_clears();
    test_query_byte_midframe_is_frame_data();
    test_query_byte_after_preamble_is_frame_data();
    test_overdrive_kick_survives_millis_wraparound();
    test_watchdog_survives_millis_wraparound();

    if (g_failures == 0) {
        std::printf("OK — all tests pass\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d test(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
