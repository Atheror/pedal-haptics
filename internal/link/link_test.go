package link

import (
	"errors"
	"testing"

	"github.com/Atheror/pedal-haptics/internal/protocol"
)

func TestHandshakeParsesVersionAndCap(t *testing.T) {
	l, err := New(NewFake("PH1 0.1.0 179\n"))
	if err != nil {
		t.Fatalf("New() error: %v", err)
	}
	if l.Version() != "0.1.0" {
		t.Errorf("Version() = %q, want %q", l.Version(), "0.1.0")
	}
	if l.DutyCap() != 179 {
		t.Errorf("DutyCap() = %d, want 179", l.DutyCap())
	}
}

func TestHandshakeRejectsUnknownFirmware(t *testing.T) {
	if _, err := New(NewFake("Arduino Leonardo ready\n")); !errors.Is(err, ErrHandshake) {
		t.Errorf("err = %v, want ErrHandshake", err)
	}
}

func TestHandshakeRejectsSilence(t *testing.T) {
	if _, err := New(NewFake("")); !errors.Is(err, ErrHandshake) {
		t.Errorf("err = %v, want ErrHandshake", err)
	}
}

func TestSendWritesEncodedFrame(t *testing.T) {
	f := NewFake("PH1 0.1.0 179\n")
	l, err := New(f)
	if err != nil {
		t.Fatalf("New() error: %v", err)
	}
	frame := protocol.Frame{Duty: [2]uint8{42, 84}}
	if err := l.Send(frame); err != nil {
		t.Fatalf("Send() error: %v", err)
	}
	want := frame.Encode()
	if len(f.Written) != protocol.FrameSize {
		t.Fatalf("wrote %d bytes, want %d", len(f.Written), protocol.FrameSize)
	}
	for i, b := range want {
		if f.Written[i] != b {
			t.Errorf("byte %d = %#x, want %#x", i, f.Written[i], b)
		}
	}
}

func TestSendPropagatesWriteError(t *testing.T) {
	f := NewFake("PH1 0.1.0 179\n")
	l, err := New(f)
	if err != nil {
		t.Fatalf("New() error: %v", err)
	}
	boom := errors.New("port disconnected")
	f.WriteErr = boom
	if err := l.Send(protocol.Frame{}); !errors.Is(err, boom) {
		t.Errorf("err = %v, want %v", err, boom)
	}
}
