// Package testcmd generates test patterns as lists of pure frames.
// It doesn't touch the clock or the port: that's up to the consumer.
package testcmd

import (
	"time"

	"github.com/Atheror/pedal-haptics/internal/protocol"
)

// sendInterval is the send period to the firmware (100 Hz, see spec §5.1).
const sendInterval = 10 * time.Millisecond

// Pattern is a sequence of frames to emit at regular intervals.
type Pattern interface {
	Frames() []protocol.Frame
	Interval() time.Duration
}

type pattern struct {
	frames []protocol.Frame
}

func (p pattern) Frames() []protocol.Frame { return p.frames }
func (p pattern) Interval() time.Duration  { return sendInterval }

// steps returns how many frames fit in d.
func steps(d time.Duration) int {
	n := int(d / sendInterval)
	if n < 1 {
		n = 1
	}
	return n
}

// frameFor builds a frame with `duty` on channel `ch` and 0 on the other.
func frameFor(ch int, duty uint8) protocol.Frame {
	var f protocol.Frame
	if ch == 0 || ch == 1 {
		f.Duty[ch] = duty
	}
	return f
}

// Hold keeps a constant duty on a channel.
func Hold(ch int, duty uint8, d time.Duration) Pattern {
	n := steps(d)
	frames := make([]protocol.Frame, n)
	for i := range frames {
		frames[i] = frameFor(ch, duty)
	}
	return pattern{frames}
}

// Sweep produces a 0 → 255 → 0 ramp. Useful for checking linearity.
// It applies no cap: the cap is the firmware's responsibility (spec §4.3), and
// this pattern exists precisely to verify the firmware applies it.
func Sweep(ch int, d time.Duration) Pattern {
	n := steps(d)
	// Minimum 4 frames: each ramp needs at least 2 points, and with fewer
	// the (up-1) and (down-1) denominators would be zero.
	if n < 4 {
		n = 4
	}
	up := n / 2
	down := n - up
	frames := make([]protocol.Frame, 0, n)
	for i := 0; i < up; i++ {
		frames = append(frames, frameFor(ch, uint8(i*255/(up-1))))
	}
	for i := 0; i < down; i++ {
		frames = append(frames, frameFor(ch, uint8(255-i*255/(down-1))))
	}
	return pattern{frames}
}

// Pulse alternates on and off at `hz`, with a 50% duty cycle.
// It's the key test for active braking: without it, at 12 Hz the pulses
// blur together from inertia and feel like continuous buzz (spec §3.2).
func Pulse(ch int, duty uint8, hz float64, d time.Duration) Pattern {
	n := steps(d)
	framesPerCycle := float64(time.Second) / float64(sendInterval) / hz
	frames := make([]protocol.Frame, n)
	for i := range frames {
		phase := float64(i) / framesPerCycle
		if phase-float64(int(phase)) < 0.5 {
			frames[i] = frameFor(ch, duty)
		} else {
			frames[i] = frameFor(ch, 0)
		}
	}
	return pattern{frames}
}
