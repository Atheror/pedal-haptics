// Package link manages the serial link with the RP2040 firmware.
package link

import (
	"errors"
	"fmt"
	"io"
	"strconv"
	"strings"

	"github.com/Atheror/pedal-haptics/internal/protocol"
)

// ErrHandshake indicates that the device on the other end is not a
// recognizable pedal-haptics firmware.
var ErrHandshake = errors.New("link: handshake failed")

// Port is the minimum Link needs. A real serial port and the test Fake
// both satisfy it equally.
type Port interface {
	io.ReadWriteCloser
}

// Link is an open connection to an already-identified firmware.
type Link struct {
	port    Port
	version string
	dutyCap uint8
}

// QueryBanner asks the firmware to (re)print its identification line.
// Spec §4.1: the firmware answers "solo al conectar y ante 0x3F".
const QueryBanner byte = 0x3F

// New opens the link and verifies the handshake. The firmware announces
// "PH1 <version> <duty_cap>" on connect. See spec §4.1.
//
// The query byte is not optional politeness. An RP2040 does not reset when
// the host opens its CDC port -- unlike an AVR board, where DTR toggles a
// reset line -- so the firmware's setup() runs once per power cycle and its
// banner is long gone by the time a second run connects. Without the query,
// every connection after the first would sit through the read timeouts and
// fail, and the symptom would look exactly like a flaky cable.
func New(p Port) (*Link, error) {
	if _, err := p.Write([]byte{QueryBanner}); err != nil {
		return nil, fmt.Errorf("%w: sending banner query: %w", ErrHandshake, err)
	}
	line, err := readLine(p, 64)
	if err != nil && line == "" {
		return nil, fmt.Errorf("%w: no response from device", ErrHandshake)
	}
	fields := strings.Fields(strings.TrimSpace(line))
	if len(fields) != 3 || fields[0] != "PH1" {
		return nil, fmt.Errorf("%w: unexpected response %q", ErrHandshake, strings.TrimSpace(line))
	}
	cap64, err := strconv.ParseUint(fields[2], 10, 8)
	if err != nil {
		return nil, fmt.Errorf("%w: unreadable duty_cap %q", ErrHandshake, fields[2])
	}
	return &Link{port: p, version: fields[1], dutyCap: uint8(cap64)}, nil
}

// Send transmits a frame.
func (l *Link) Send(f protocol.Frame) error {
	b := f.Encode()
	_, err := l.port.Write(b[:])
	return err
}

// Version returns the firmware version announced in the handshake.
func (l *Link) Version() string { return l.version }

// DutyCap returns the duty cap applied by the firmware. Informational: the
// actual cap is enforced by the MCU, not the host. See spec §4.3.
func (l *Link) DutyCap() uint8 { return l.dutyCap }

// Close closes the underlying port.
func (l *Link) Close() error { return l.port.Close() }

// readLine reads byte by byte until '\n' or up to max bytes.
//
// Deliberately doesn't use bufio.ReadString: a real serial port with a read
// timeout returns (0, nil) on expiry, and bufio would retry indefinitely
// against a silent board. Here a streak of empty reads aborts.
func readLine(p Port, max int) (string, error) {
	var buf []byte
	one := make([]byte, 1)
	empties := 0
	for len(buf) < max {
		n, err := p.Read(one)
		if err != nil {
			return string(buf), err
		}
		if n == 0 {
			empties++
			if empties > 3 {
				return string(buf), io.ErrUnexpectedEOF
			}
			continue
		}
		empties = 0
		if one[0] == '\n' {
			return string(buf), nil
		}
		buf = append(buf, one[0])
	}
	return string(buf), nil
}
