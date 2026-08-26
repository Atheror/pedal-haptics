package link

import (
	"bytes"
	"sync"
)

// Fake is an in-memory Port for tests. It returns `reply` on read and
// accumulates everything written into Written.
type Fake struct {
	mu      sync.Mutex
	reply   *bytes.Reader
	Written []byte
	Closed  bool
	// WriteErr, if not nil, is returned on every Write.
	WriteErr error
}

func NewFake(reply string) *Fake {
	return &Fake{reply: bytes.NewReader([]byte(reply))}
}

func (f *Fake) Read(p []byte) (int, error) {
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
