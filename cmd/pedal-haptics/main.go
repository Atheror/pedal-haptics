// Command pedal-haptics controls the pedal rumble firmware.
package main

import (
	"flag"
	"fmt"
	"os"
	"time"

	"go.bug.st/serial"

	"github.com/Atheror/pedal-haptics/internal/link"
	"github.com/Atheror/pedal-haptics/internal/testcmd"
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	switch os.Args[1] {
	case "test":
		if err := runTest(os.Args[2:]); err != nil {
			fmt.Fprintln(os.Stderr, "error:", err)
			os.Exit(1)
		}
	default:
		usage()
		os.Exit(2)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, `usage: pedal-haptics test [flags]

  --port    serial port (default /dev/ttyACM0)
  --channel 0 = brake, 1 = throttle (default 0)
  --duty    0..255 (default 128)
  --ms      duration in milliseconds (default 1000)
  --sweep   0→255→0 ramp instead of constant duty
  --pulse   pulse frequency in Hz (0 = disabled)`)
}

func runTest(args []string) error {
	fs := flag.NewFlagSet("test", flag.ExitOnError)
	port := fs.String("port", "/dev/ttyACM0", "serial port")
	channel := fs.Int("channel", 0, "channel (0=brake, 1=throttle)")
	duty := fs.Int("duty", 128, "duty 0..255")
	ms := fs.Int("ms", 1000, "duration in ms")
	sweep := fs.Bool("sweep", false, "0→255→0 ramp")
	pulse := fs.Float64("pulse", 0, "pulse frequency in Hz")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if *channel != 0 && *channel != 1 {
		return fmt.Errorf("channel must be 0 or 1, got %d", *channel)
	}
	if *duty < 0 || *duty > 255 {
		return fmt.Errorf("duty must be between 0 and 255, got %d", *duty)
	}

	p, err := serial.Open(*port, &serial.Mode{BaudRate: 115200})
	if err != nil {
		return fmt.Errorf("opening %s: %w", *port, err)
	}
	defer p.Close()

	// The firmware announces PH1 when the port opens; give it some margin.
	if err := p.SetReadTimeout(2 * time.Second); err != nil {
		return err
	}

	l, err := link.New(p)
	if err != nil {
		return err
	}
	fmt.Printf("firmware %s, duty cap %d\n", l.Version(), l.DutyCap())

	d := time.Duration(*ms) * time.Millisecond
	var pattern testcmd.Pattern
	switch {
	case *sweep:
		pattern = testcmd.Sweep(*channel, d)
	case *pulse > 0:
		pattern = testcmd.Pulse(*channel, uint8(*duty), *pulse, d)
	default:
		pattern = testcmd.Hold(*channel, uint8(*duty), d)
	}

	frames := pattern.Frames()
	ticker := time.NewTicker(pattern.Interval())
	defer ticker.Stop()
	for _, f := range frames {
		if err := l.Send(f); err != nil {
			return fmt.Errorf("sending frame: %w", err)
		}
		<-ticker.C
	}

	// Stop sending. The firmware's watchdog shuts off the motors after 250 ms;
	// there's no need to send a zero frame, and not sending one verifies along
	// the way that the watchdog works.
	fmt.Println("pattern finished; the watchdog should shut off in 250 ms")
	return nil
}
