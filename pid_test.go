package viamboatbase

import (
	"testing"
	"time"

	"go.viam.com/test"
)

func TestPID1(t *testing.T) {

	targetSpeed := 5.0
	currentSpeed := 0.0

	pid := pidState{}
	pid.setDefaults()

	dt := time.Millisecond * 100

	for i := 0; i < 1000; i++ {
		motorPower := pid.Control(targetSpeed, currentSpeed, dt)
		currentSpeed = motorPower * 10

		if i > 200 { // TODO(erh) how soon do we converge
			test.That(t, currentSpeed, test.ShouldAlmostEqual, targetSpeed, .01)
		}
	}

}
