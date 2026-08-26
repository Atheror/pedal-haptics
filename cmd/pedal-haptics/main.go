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

const (
	defaultPort = "/dev/ttyACM0"

	// maxPulseHz is the highest pulse frequency the frame rate can actually
	// express. One full cycle needs at least two frames -- one on, one off --
	// so at testcmd.SendInterval (100 Hz) anything above 50 Hz collapses into
	// a constant hold, and values in between alias down to some unrelated
	// frequency (--pulse 66 comes out at ~33 Hz). --pulse exists to judge
	// whether active braking makes pulses feel discrete, so a silent aliasing
	// artifact there would be misread as a hardware limitation.
	maxPulseHz = float64(time.Second) / float64(2*testcmd.SendInterval)
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
	fmt.Fprintf(os.Stderr, `usage: pedal-haptics test [flags]

  --port    serial port (default %s)
  --channel 0 = brake, 1 = throttle (default 0)
  --duty    0..255 (default 128); not valid with --sweep
  --ms      duration in milliseconds (default 1000)
  --sweep   0→255→0 ramp instead of constant duty
  --pulse   pulse frequency in Hz, 0 < hz <= %g (omit to disable)
`, defaultPort, maxPulseHz)
}

func runTest(args []string) error {
	fs := flag.NewFlagSet("test", flag.ExitOnError)
	port := fs.String("port", defaultPort, "serial port")
	channel := fs.Int("channel", 0, "channel (0=brake, 1=throttle)")
	duty := fs.Int("duty", 128, "duty 0..255")
	ms := fs.Int("ms", 1000, "duration in ms")
	sweep := fs.Bool("sweep", false, "0→255→0 ramp")
	pulse := fs.Float64("pulse", 0, "pulse frequency in Hz")
	if err := fs.Parse(args); err != nil {
		return err
	}

	// Which flags the user actually typed, so that "left at its default" and
	// "explicitly asked for" can be told apart.
	given := map[string]bool{}
	fs.Visit(func(f *flag.Flag) { given[f.Name] = true })

	if *channel != 0 && *channel != 1 {
		return fmt.Errorf("channel must be 0 or 1, got %d", *channel)
	}
	if *duty < 0 || *duty > 255 {
		return fmt.Errorf("duty must be between 0 and 255, got %d", *duty)
	}
	if *ms <= 0 {
		return fmt.Errorf("ms must be positive, got %d", *ms)
	}
	// Written as a positive range so NaN, negatives and 0 are all rejected
	// rather than quietly falling through to the Hold branch.
	if given["pulse"] && !(*pulse > 0 && *pulse <= maxPulseHz) {
		return fmt.Errorf("pulse must be greater than 0 and at most %g Hz, got %g: "+
			"frames go out at %g Hz, and a cycle needs one frame on and one off, "+
			"so anything faster aliases into a hold",
			maxPulseHz, *pulse, float64(time.Second)/float64(testcmd.SendInterval))
	}
	// --sweep used to silently ignore both of these.
	if *sweep && given["duty"] {
		return fmt.Errorf("--duty does not apply to --sweep: the sweep ramps 0→255→0 " +
			"by design, and the firmware caps it (spec §4.3)")
	}
	if *sweep && given["pulse"] {
		return fmt.Errorf("--sweep and --pulse are different patterns; pick one")
	}

	p, err := serial.Open(*port, &serial.Mode{BaudRate: 115200})
	if err != nil {
		return fmt.Errorf("opening %s: %w", *port, err)
	}
	defer p.Close()

	// link.New queries the firmware for its PH1 banner; give it some margin.
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
