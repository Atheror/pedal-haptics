// Package protocol implementa el códec de la trama de cable entre el daemon y
// el firmware RP2040. Ver spec §4.1.
package protocol

import "errors"

const (
	// Preamble marca el inicio de una trama.
	Preamble byte = 0xA5
	// FrameSize es el tamaño fijo de una trama en bytes.
	FrameSize = 5
)

// Flags de override. El frenado normal es responsabilidad del firmware al
// detectar duty 0; estos bits fuerzan freno inmediato. Ver spec §4.1.
const (
	FlagBrakeCh0 byte = 1 << 0
	FlagBrakeCh1 byte = 1 << 1
)

var (
	ErrShortFrame  = errors.New("protocol: trama incompleta")
	ErrBadPreamble = errors.New("protocol: preámbulo inválido")
	ErrBadChecksum = errors.New("protocol: checksum inválido")
)

// Frame es una orden para los dos canales. Índice 0 = freno, 1 = acelerador.
type Frame struct {
	Duty  [2]uint8
	Flags uint8
}

// Encode serializa la trama con su checksum XOR.
func (f Frame) Encode() [FrameSize]byte {
	var b [FrameSize]byte
	b[0] = Preamble
	b[1] = f.Duty[0]
	b[2] = f.Duty[1]
	b[3] = f.Flags
	b[4] = b[0] ^ b[1] ^ b[2] ^ b[3]
	return b
}

// Decode valida y deserializa una trama.
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
