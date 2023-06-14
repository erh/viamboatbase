package viamboatbase

import (
	"time"
)

type pidState struct {
	// config
	proportionalGain float64
	integralGain     float64
	derivativeGain   float64

	minOutput, maxOutput float64

	// state
	integral      float64
	previousError float64
}

func (pid *pidState) setDefaults() {
	pid.proportionalGain = 0.08
	pid.integralGain = 0.075
	pid.derivativeGain = 0.0001

	pid.minOutput = -1
	pid.maxOutput = 1
}

func (pid *pidState) Control(target, current float64, timeSinceLastCall time.Duration) float64 {

	error := target - current

	p := pid.proportionalGain * error

	pid.integral += error * timeSinceLastCall.Seconds()
	i := pid.integralGain * pid.integral

	d := pid.derivativeGain * (error - pid.previousError) / timeSinceLastCall.Seconds()
	pid.previousError = error

	n := p + i + d

	if pid.minOutput != 0 && n < pid.minOutput {
		n = pid.minOutput
	}

	if pid.maxOutput != 0 && n > pid.maxOutput {
		n = pid.maxOutput
	}

	return n
}
