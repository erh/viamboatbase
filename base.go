package viamboatbase

import (
	"context"
	"errors"
	"math"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.uber.org/multierr"
	"go.viam.com/utils"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/motor"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
)

var Model = resource.ModelNamespace("erh").WithFamily("base").WithModel("boat")

const pidLoopTime = time.Millisecond * 500

func init() {
	boatComp := resource.Registration[base.Base, *Config]{
		Constructor: func(
			ctx context.Context, deps resource.Dependencies, conf resource.Config, logger golog.Logger,
		) (base.Base, error) {
			return createBoat(deps, conf, logger)
		},
	}
	resource.RegisterComponent(base.API, Model, boatComp)
}

func createBoat(deps resource.Dependencies, conf resource.Config, logger golog.Logger) (base.LocalBase, error) {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	theBoat := &boat{
		Named:  conf.ResourceName().AsNamed(),
		cfg:    newConf,
		logger: logger,
	}

	theBoat.state.angularPID.setDefaults()
	theBoat.state.linearPID.setDefaults()

	for _, mc := range newConf.Motors {
		m, err := motor.FromDependencies(deps, mc.Name)
		if err != nil {
			return nil, err
		}
		theBoat.motors = append(theBoat.motors, m)
	}

	if newConf.MovementSensor != "" {
		var err error
		theBoat.movementSensor, err = movementsensor.FromDependencies(deps, newConf.MovementSensor)
		if err != nil {
			return nil, err
		}
	}
	return theBoat, nil
}

type controlMode int

const (
	controlNone     controlMode = 0
	controlVelocity             = 1
	controlHeading              = 2
)

type boatState struct {
	threadStarted bool
	controlState  controlMode

	angularPID, linearPID                   pidState
	velocityLinearGoal, velocityAngularGoal r3.Vector

	compassGoal  float64
	spinVelocity float64
}

type boat struct {
	resource.Named
	resource.AlwaysRebuild

	cfg            *Config
	motors         []motor.Motor
	movementSensor movementsensor.MovementSensor

	opMgr operation.SingleOperationManager

	state      boatState
	stateMutex sync.Mutex

	cancel    context.CancelFunc
	waitGroup sync.WaitGroup

	logger golog.Logger
}

func (b *boat) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	if distanceMm < 0 {
		mmPerSec *= -1
		distanceMm *= -1
	}
	err := b.SetVelocity(ctx, r3.Vector{Y: mmPerSec}, r3.Vector{}, extra)
	if err != nil {
		return err
	}
	s := time.Duration(float64(time.Millisecond) * math.Abs(float64(distanceMm)))
	utils.SelectContextOrWait(ctx, s)
	return b.Stop(ctx, nil)
}

func (b *boat) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	if b.movementSensor == nil {
		return errors.New("no movementSensor")
	}

	compass, err := b.movementSensor.CompassHeading(ctx, nil)
	if err != nil {
		return err
	}

	goal := compass + angleDeg

	b.logger.Infof("Spin angleDeg: %v degsPerSec: %v compass: %v goal: %v", angleDeg, degsPerSec, compass, goal)
	_, done := b.opMgr.New(ctx)
	defer done()

	b.stateMutex.Lock()

	b.state.controlState = controlHeading
	b.state.compassGoal = goal
	b.state.velocityLinearGoal = r3.Vector{}
	b.state.spinVelocity = degsPerSec
	b.state.velocityAngularGoal = r3.Vector{0, 0, 0}

	err = b.startVelocityThreadInLock()

	b.stateMutex.Unlock()

	if err != nil {
		return err
	}

	return b.opMgr.WaitForSuccess(ctx, time.Second, func(ctx context.Context) (bool, error) {
		compass, err := b.movementSensor.CompassHeading(ctx, nil)
		if err != nil {
			return false, err
		}

		return rdkutils.AngleDiffDeg(goal, compass) < 1, nil
	})
}

func (b *boat) startVelocityThreadInLock() error {
	if b.state.threadStarted {
		return nil
	}

	if b.movementSensor == nil {
		return errors.New("no movementSensor")
	}

	var ctx context.Context
	ctx, b.cancel = context.WithCancel(context.Background())

	b.waitGroup.Add(1)
	go func() {
		defer b.waitGroup.Done()

		for {
			utils.SelectContextOrWait(ctx, pidLoopTime)
			err := b.velocityThreadLoop(ctx)
			if err != nil {
				if errors.Is(err, context.Canceled) {
					return
				}
				b.logger.Warn(err)
			}
		}
	}()
	b.state.threadStarted = true
	return nil
}

func (b *boat) velocityThreadLoop(ctx context.Context) error {
	// TODO(erh) optimize how we get all sensor stuff

	av, err := b.movementSensor.AngularVelocity(ctx, make(map[string]interface{}))
	if err != nil {
		return err
	}

	lv, err := b.movementSensor.LinearVelocity(ctx, make(map[string]interface{}))
	if err != nil {
		return err
	}

	heading, err := b.movementSensor.CompassHeading(ctx, nil)
	if err != nil {
		return err
	}

	// ------

	b.stateMutex.Lock()
	if b.state.controlState == controlNone {
		b.stateMutex.Unlock()
		return nil
	}

	var linear, angular r3.Vector

	if b.state.controlState == controlVelocity {
		linear, angular = computeNextPower(&b.state, lv, av, b.logger)
	} else if b.state.controlState == controlHeading {
		updateVelocityGoalForHeading(&b.state, heading)
		b.logger.Infof("heading control compass: %v goal: %v angular z: %v", heading, b.state.compassGoal, b.state.velocityAngularGoal.Z)
		linear, angular = computeNextPower(&b.state, lv, av, b.logger)
	}

	b.stateMutex.Unlock()

	return b.setPowerInternal(ctx, linear, angular)
}

func updateVelocityGoalForHeading(state *boatState, heading float64) {
	diff := heading - state.compassGoal
	if diff < -5 {
		state.velocityAngularGoal.Z = -1 * state.spinVelocity
	} else if diff > 5 {
		state.velocityAngularGoal.Z = state.spinVelocity
	} else if diff < -1 {
		state.velocityAngularGoal.Z = (diff * -1 / 5) * state.spinVelocity
	} else if diff > 1 {
		state.velocityAngularGoal.Z = (diff / 5) * state.spinVelocity
	} else {
		state.velocityAngularGoal.Z = 0
	}
}

func computeNextPower(
	state *boatState,
	linearVelocity r3.Vector,
	angularVelocity spatialmath.AngularVelocity,
	logger golog.Logger) (r3.Vector, r3.Vector) {

	return r3.Vector{0, state.linearPID.Control(state.velocityLinearGoal.Y, linearVelocity.Y, pidLoopTime), 0},
		r3.Vector{0, 0, state.angularPID.Control(state.velocityAngularGoal.Z, angularVelocity.Z, pidLoopTime)}
}

func (b *boat) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	b.logger.Debugf("SetVelocity %v %v", linear, angular)
	_, done := b.opMgr.New(ctx)
	defer done()

	b.stateMutex.Lock()
	defer b.stateMutex.Unlock()

	err := b.startVelocityThreadInLock()
	if err != nil {
		return err
	}

	b.state.controlState = controlVelocity
	b.state.velocityLinearGoal = linear
	b.state.velocityAngularGoal = angular

	return nil
}

func (b *boat) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	b.logger.Debugf("SetPower %v %v", linear, angular)
	ctx, done := b.opMgr.New(ctx)
	defer done()

	b.stateMutex.Lock()
	b.state.controlState = controlNone
	b.stateMutex.Unlock()

	return b.setPowerInternal(ctx, linear, angular)
}

func (b *boat) setPowerInternal(ctx context.Context, linear, angular r3.Vector) error {
	power, err := b.cfg.ComputePower(linear, angular)
	if err != nil {
		return err
	}

	for idx, p := range power {
		err := b.motors[idx].SetPower(ctx, p, nil)
		if err != nil {
			return multierr.Combine(b.Stop(ctx, nil), err)
		}
		if ctx.Err() != nil {
			return ctx.Err()
		}
	}

	return nil
}

func (b *boat) Stop(ctx context.Context, extra map[string]interface{}) error {
	b.stateMutex.Lock()
	b.state.velocityLinearGoal = r3.Vector{}
	b.state.velocityAngularGoal = r3.Vector{}
	b.stateMutex.Unlock()

	b.opMgr.CancelRunning(ctx)
	var err error
	for _, m := range b.motors {
		err = multierr.Combine(m.Stop(ctx, nil), err)
	}
	return err
}

func (b *boat) Width(ctx context.Context) (int, error) {
	return int(b.cfg.WidthMM), nil
}

func (b *boat) IsMoving(ctx context.Context) (bool, error) {
	for _, m := range b.motors {
		isMoving, _, err := m.IsPowered(ctx, nil)
		if err != nil {
			return false, err
		}
		if isMoving {
			return true, err
		}
	}
	return false, nil
}

func (b *boat) Close(ctx context.Context) error {
	if b.cancel != nil {
		b.cancel()
		b.cancel = nil
		b.waitGroup.Wait()
	}
	return b.Stop(ctx, nil)
}
