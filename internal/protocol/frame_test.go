package protocol

import "testing"

func TestEncodeLayout(t *testing.T) {
	f := Frame{Duty: [2]uint8{10, 20}, Flags: FlagBrakeCh1}
	b := f.Encode()

	if b[0] != Preamble {
		t.Errorf("preamble = %#x, want %#x", b[0], Preamble)
	}
	if b[1] != 10 || b[2] != 20 {
		t.Errorf("duty = %d,%d, want 10,20", b[1], b[2])
	}
	if b[3] != FlagBrakeCh1 {
		t.Errorf("flags = %#x, want %#x", b[3], FlagBrakeCh1)
	}
	want := b[0] ^ b[1] ^ b[2] ^ b[3]
	if b[4] != want {
		t.Errorf("checksum = %#x, want %#x", b[4], want)
	}
}

func TestRoundTrip(t *testing.T) {
	cases := []Frame{
		{Duty: [2]uint8{0, 0}},
		{Duty: [2]uint8{255, 255}, Flags: FlagBrakeCh0 | FlagBrakeCh1},
		{Duty: [2]uint8{1, 254}, Flags: FlagBrakeCh0},
	}
	for _, in := range cases {
		b := in.Encode()
		out, err := Decode(b[:])
		if err != nil {
			t.Fatalf("Decode(%v) error: %v", in, err)
		}
		if out != in {
			t.Errorf("round trip: got %v, want %v", out, in)
		}
	}
}

func TestDecodeRejectsShortFrame(t *testing.T) {
	if _, err := Decode([]byte{0xA5, 0, 0, 0}); err != ErrShortFrame {
		t.Errorf("err = %v, want ErrShortFrame", err)
	}
}

func TestDecodeRejectsBadPreamble(t *testing.T) {
	f := Frame{Duty: [2]uint8{5, 5}}
	b := f.Encode()
	b[0] = 0x00
	b[4] = b[0] ^ b[1] ^ b[2] ^ b[3] // checksum valid, preamble bad
	if _, err := Decode(b[:]); err != ErrBadPreamble {
		t.Errorf("err = %v, want ErrBadPreamble", err)
	}
}

func TestDecodeRejectsCorruptedByte(t *testing.T) {
	f := Frame{Duty: [2]uint8{100, 100}}
	b := f.Encode()
	b[1] ^= 0xFF
	if _, err := Decode(b[:]); err != ErrBadChecksum {
		t.Errorf("err = %v, want ErrBadChecksum", err)
	}
}
