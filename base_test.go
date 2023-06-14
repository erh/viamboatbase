package viamboatbase

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/spatialmath"
)

func TestComputeNextPower(t *testing.T) {
	state := &boatState{
		velocityAngularGoal: r3.Vector{Z: 5},
	}
	state.angularPID.setDefaults()
	state.linearPID.setDefaults()

	_, a := computeNextPower(
		state,
		r3.Vector{},
		spatialmath.AngularVelocity{},
		nil,
	)
	test.That(t, a.Z, test.ShouldAlmostEqual, .588, .01)
}
