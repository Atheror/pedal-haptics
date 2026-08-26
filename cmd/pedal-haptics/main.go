// Command pedal-haptics controls the pedal rumble firmware.
package main

import (
	"flag"
	"fmt"
	"os"
	"time"

	"go.bug.st/serial"

	"github.com/Atheror/pedal-haptics/internal/link"
	"github.com/Atheror/pedal-haptics/internal/protocol"
	"github.com/Atheror/pedal-haptics/internal/testcmd"
)

const (
	defaultPort = "/dev/ttyACM0"
	baudRate    = 115200

	// maxPulseHz is the highest pulse frequency the frame rate can actually
	// express. One full cycle needs at least two frames -- one on, one off --
	// so at testcmd.SendInterval (100 Hz) anything above 50 Hz collapses into
	// a constant hold or aliases down to some unrelated frequency. --pulse
	// exists to judge whether active braking makes pulses feel discrete, so a
	// silent aliasing artifact there would be misread as a hardware limit.
	maxPulseHz = float64(time.Second) / float64(2*testcmd.SendInterval)

	// stopFrames is how many emergency-stop frames `stop` sends, one per
	// SendInterval. More than one because a single frame lost to noise would
	// leave the motors running until the 250 ms watchdog caught up.
	stopFrames = 5
)

func main() {
	if len(os.Args) < 2 {
		usage()
		os.Exit(2)
	}
	var err error
	switch os.Args[1] {
	case "test":
		err = runTest(os.Args[2:])
	case "stop":
		err = runStop(os.Args[2:])
	default:
		usage()
		os.Exit(2)
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}

func usage() {
	fmt.Fprintf(os.Stderr, `usage: pedal-haptics <command> [flags]

commands:
  test    run a test pattern on one channel
  stop    emergency stop: brake both channels immediately

test flags:
  --port    serial port (default %s)
  --channel 0 = brake, 1 = throttle (default 0)
  --duty    0..255 (default 128); not valid with --sweep
  --ms      duration in milliseconds (default 1000)
  --sweep   0→255→0 ramp instead of constant duty
  --pulse   pulse frequency in Hz, 0 < hz <= %g (omit to disable)

stop flags:
  --port    serial port (default %s)
`, defaultPort, maxPulseHz, defaultPort)
}

// connect opens the port and completes the PH1 handshake. The caller owns the
// returned Link and must Close it.
func connect(port string) (*link.Link, error) {
	p, err := serial.Open(port, &serial.Mode{BaudRate: baudRate})
	if err != nil {
		return nil, fmt.Errorf("opening %s: %w", port, err)
	}
	// link.New queries the firmware for its PH1 banner; give it some margin.
	if err := p.SetReadTimeout(2 * time.Second); err != nil {
		p.Close()
		return nil, err
	}
	l, err := link.New(p)
	if err != nil {
		p.Close()
		return nil, err
	}
	return l, nil
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

	l, err := connect(*port)
	if err != nil {
		return err
	}
	defer l.Close()
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

// runStop is the emergency stop. Spec §4.1 names it as one of the two reasons
// the brake bits exist: during bring-up with motors strapped to the pedals,
// the alternative is Ctrl-C plus 250 ms of watchdog, or pulling the cable.
func runStop(args []string) error {
	fs := flag.NewFlagSet("stop", flag.ExitOnError)
	port := fs.String("port", defaultPort, "serial port")
	if err := fs.Parse(args); err != nil {
		return err
	}

	l, err := connect(*port)
	if err != nil {
		return err
	}
	defer l.Close()
	fmt.Printf("firmware %s, duty cap %d\n", l.Version(), l.DutyCap())

	// Both brake bits, duty 0. The firmware zeroes the channel and holds it
	// braked; it does not wait for the watchdog.
	f := protocol.Frame{Flags: protocol.FlagBrakeCh0 | protocol.FlagBrakeCh1}
	ticker := time.NewTicker(testcmd.SendInterval)
	defer ticker.Stop()
	for i := 0; i < stopFrames; i++ {
		if err := l.Send(f); err != nil {
			return fmt.Errorf("sending stop frame %d of %d: %w", i+1, stopFrames, err)
		}
		<-ticker.C
	}

	fmt.Printf("sent %d brake frames on both channels over %v; "+
		"motors braked, and the watchdog holds them there\n",
		stopFrames, time.Duration(stopFrames)*testcmd.SendInterval)
	return nil
}
