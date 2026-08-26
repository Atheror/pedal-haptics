package testcmd

import (
	"testing"
	"time"
)

const tick = 10 * time.Millisecond // 100 Hz, see Global Constraints

func TestHoldProducesConstantDuty(t *testing.T) {
	p := Hold(0, 128, 100*time.Millisecond)
	frames := p.Frames()
	if len(frames) != 10 {
		t.Fatalf("len(frames) = %d, want 10", len(frames))
	}
	for i, f := range frames {
		if f.Duty[0] != 128 {
			t.Errorf("frame %d: Duty[0] = %d, want 128", i, f.Duty[0])
		}
		if f.Duty[1] != 0 {
			t.Errorf("frame %d: Duty[1] = %d, want 0 (channel not selected)", i, f.Duty[1])
		}
	}
}

func TestHoldRespectsChannel(t *testing.T) {
	frames := Hold(1, 200, 20*time.Millisecond).Frames()
	if frames[0].Duty[1] != 200 {
		t.Errorf("Duty[1] = %d, want 200", frames[0].Duty[1])
	}
	if frames[0].Duty[0] != 0 {
		t.Errorf("Duty[0] = %d, want 0", frames[0].Duty[0])
	}
}

func TestSweepGoesUpThenDown(t *testing.T) {
	frames := Sweep(0, 200*time.Millisecond).Frames()
	if len(frames) < 4 {
		t.Fatalf("len(frames) = %d, too short for a ramp", len(frames))
	}
	first, last := frames[0].Duty[0], frames[len(frames)-1].Duty[0]
	if first != 0 {
		t.Errorf("first duty = %d, want 0", first)
	}
	if last != 0 {
		t.Errorf("last duty = %d, want 0", last)
	}

	// The ascending ramp must reach 255 on its own. A global peak check is not
	// enough: the descending ramp's first sample is 255 by construction, so it
	// would mask a broken ascending denominator (e.g. i*255/up instead of
	// i*255/(up-1), which tops out at 229 for up=10).
	up := len(frames) / 2
	if frames[up-1].Duty[0] != 255 {
		t.Errorf("last ascending duty = %d, want 255", frames[up-1].Duty[0])
	}
	for i := 1; i < up; i++ {
		if frames[i].Duty[0] < frames[i-1].Duty[0] {
			t.Errorf("ascending ramp not monotonic at %d: %d < %d",
				i, frames[i].Duty[0], frames[i-1].Duty[0])
		}
	}

	var peak uint8
	for _, f := range frames {
		if f.Duty[0] > peak {
			peak = f.Duty[0]
		}
	}
	if peak != 255 {
		t.Errorf("peak = %d, want 255 (the cap is applied by the firmware, not the host)", peak)
	}
}

func TestPulseAlternatesOnAndOff(t *testing.T) {
	// 10 Hz for 1 s = 10 cycles. At a 100 Hz send rate, 10 frames per cycle:
	// 5 on and 5 off.
	frames := Pulse(0, 255, 10, time.Second).Frames()
	if len(frames) != 100 {
		t.Fatalf("len(frames) = %d, want 100", len(frames))
	}

	var on int
	for _, f := range frames {
		if f.Duty[0] > 0 {
			on++
		}
	}
	if on < 45 || on > 55 {
		t.Errorf("frames on = %d, want ~50 (50%% duty cycle)", on)
	}

	// The first edge should fall at the midpoint of the cycle: frames 0-4 on, 5-9 off.
	if frames[0].Duty[0] == 0 {
		t.Error("frame 0 should be on")
	}
	if frames[5].Duty[0] != 0 {
		t.Error("frame 5 should be off")
	}
}

func TestIntervalIsSendRate(t *testing.T) {
	if got := Hold(0, 10, time.Second).Interval(); got != tick {
		t.Errorf("Interval() = %v, want %v", got, tick)
	}
}

func TestSweepHandlesVeryShortDuration(t *testing.T) {
	// Must not panic on division by zero.
	frames := Sweep(0, 10*time.Millisecond).Frames()
	if len(frames) < 4 {
		t.Errorf("len(frames) = %d, want >= 4", len(frames))
	}
	if frames[0].Duty[0] != 0 {
		t.Errorf("first duty = %d, want 0", frames[0].Duty[0])
	}
	if frames[len(frames)-1].Duty[0] != 0 {
		t.Errorf("last duty = %d, want 0", frames[len(frames)-1].Duty[0])
	}
}
