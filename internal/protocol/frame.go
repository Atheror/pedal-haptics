// Package protocol implements the wire frame codec between the daemon and
// the RP2040 firmware. See spec §4.1.
package protocol

import "errors"

const (
	// Preamble marks the start of a frame.
	Preamble byte = 0xA5
	// FrameSize is the fixed size of a frame in bytes.
	FrameSize = 5
)

// Override flags. Normal braking is the firmware's responsibility when it
// detects duty 0; these bits force an immediate brake. See spec §4.1.
const (
	FlagBrakeCh0 byte = 1 << 0
	FlagBrakeCh1 byte = 1 << 1
)

var (
	ErrShortFrame  = errors.New("protocol: incomplete frame")
	ErrBadPreamble = errors.New("protocol: invalid preamble")
	ErrBadChecksum = errors.New("protocol: invalid checksum")
)

// Frame is a command for the two channels. Index 0 = brake, 1 = throttle.
type Frame struct {
	Duty  [2]uint8
	Flags uint8
}

// Encode serializes the frame with its XOR checksum.
func (f Frame) Encode() [FrameSize]byte {
	var b [FrameSize]byte
	b[0] = Preamble
	b[1] = f.Duty[0]
	b[2] = f.Duty[1]
	b[3] = f.Flags
	b[4] = b[0] ^ b[1] ^ b[2] ^ b[3]
	return b
}

// Decode validates and deserializes a frame.
func Decode(b []byte) (Frame, error) {
	if len(b) < FrameSize {
		return Frame{}, ErrShortFrame
	}
	if b[4] != b[0]^b[1]^b[2]^b[3] {
		return Frame{}, ErrBadChecksum
	}
	if b[0] != Preamble {
		return Frame{}, ErrBadPreamble
	}
	return Frame{Duty: [2]uint8{b[1], b[2]}, Flags: b[3]}, nil
}
