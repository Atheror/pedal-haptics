package link

import (
	"bytes"
	"sync"
)

// Fake is an in-memory Port for tests. It returns `reply` on read and
// accumulates everything written into Written.
//
// Written records every byte, the handshake's QueryBanner included, because
// that byte is part of what a real port sees. Tests that only care about
// frames should clear Written after New.
type Fake struct {
	mu      sync.Mutex
	reply   *bytes.Reader
	Written []byte
	Closed  bool
	// WriteErr, if not nil, is returned on every Write.
	WriteErr error
	// EmptyReads, if > 0, makes Read return (0, nil) that many times before
	// serving reply. A real serial port does this when its read timeout
	// expires while the board stays mute -- bytes.Reader never does, so
	// without this knob readLine's give-up path is unreachable from tests.
	EmptyReads int
}

func NewFake(reply string) *Fake {
	return &Fake{reply: bytes.NewReader([]byte(reply))}
}

func (f *Fake) Read(p []byte) (int, error) {
	if f.EmptyReads > 0 {
		f.EmptyReads--
		return 0, nil
	}
	return f.reply.Read(p)
}

func (f *Fake) Write(p []byte) (int, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.WriteErr != nil {
		return 0, f.WriteErr
	}
	f.Written = append(f.Written, p...)
	return len(p), nil
}

func (f *Fake) Close() error {
	f.Closed = true
	return nil
}
