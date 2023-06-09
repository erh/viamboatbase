package viamboatbase

import (
	"fmt"
	"math"

	"github.com/go-nlopt/nlopt"
	"github.com/golang/geo/r3"
	"go.uber.org/multierr"
	"gonum.org/v1/gonum/mat"

	"go.viam.com/utils"
)

type Config struct {
	Motors         []MotorConfig
	LengthMM       float64 `json:"length_mm"`
	WidthMM        float64 `json:"width_mm"`
	MovementSensor string  `json:"movement_sensor"`
}

func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.WidthMM <= 0 {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "width_mm")
	}

	if cfg.LengthMM <= 0 {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "length_mm")
	}

	var deps []string

	if cfg.MovementSensor != "" {
		deps = append(deps, cfg.MovementSensor)
	}

	for _, m := range cfg.Motors {
		deps = append(deps, m.Name)
	}

	return deps, nil
}

func (cfg *Config) maxWeights() motorWeights {
	var max motorWeights
	for _, mc := range cfg.Motors {
		w := mc.computeWeights(math.Hypot(cfg.WidthMM, cfg.LengthMM))
		max.linearX += math.Abs(w.linearX)
		max.linearY += math.Abs(w.linearY)
		max.angular += math.Abs(w.angular)
	}
	return max
}

// examples:
//    currentVal=2 otherVal=1, currentGoal=1, otherGoal=1 = 1
//    currentVal=-2 otherVal=1, currentGoal=1, otherGoal=1 = -1

func goalScale(currentVal, otherVal, currentGoal, otherGoal float64) float64 {
	// near 0, do nothing
	if math.Abs(currentGoal) < .05 || math.Abs(otherGoal) < .05 {
		return currentVal
	}

	ratioGoal := math.Abs(currentGoal / otherGoal)
	ratioCur := math.Abs(currentVal / otherVal)

	if ratioCur > ratioGoal {
		currentVal = otherVal * ratioGoal
	}

	return currentVal
}

func (cfg *Config) computeGoal(linear, angular r3.Vector) motorWeights {
	w := cfg.maxWeights()
	w.linearX *= linear.X
	w.linearY *= linear.Y
	w.angular *= angular.Z

	w.linearX = goalScale(w.linearX, w.linearY, linear.X, linear.Y)
	w.linearY = goalScale(w.linearY, w.linearX, linear.Y, linear.X)

	// we ignore angular as the ratios don't really make sense there

	return w
}

func (cfg *Config) weights() []motorWeights {
	res := make([]motorWeights, len(cfg.Motors))
	for idx, mc := range cfg.Motors {
		w := mc.computeWeights(math.Hypot(cfg.WidthMM, cfg.LengthMM))
		res[idx] = w
	}
	return res
}

func (cfg *Config) weightsAsMatrix() *mat.Dense {
	m := mat.NewDense(3, len(cfg.Motors), nil)

	for idx, w := range cfg.weights() {
		m.Set(0, idx, w.linearX)
		m.Set(1, idx, w.linearY)
		m.Set(2, idx, w.angular)
	}

	return m
}

func (cfg *Config) ComputePowerOutputAsMatrix(powers []float64) mat.Dense {
	if len(powers) != len(cfg.Motors) {
		panic(fmt.Errorf("powers wrong length got: %d should be: %d", len(powers), len(cfg.Motors)))
	}
	var out mat.Dense

	out.Mul(cfg.weightsAsMatrix(), mat.NewDense(len(powers), 1, powers))

	return out
}

func (cfg *Config) ComputePowerOutput(powers []float64) motorWeights {
	out := cfg.ComputePowerOutputAsMatrix(powers)

	return motorWeights{
		linearX: out.At(0, 0),
		linearY: out.At(1, 0),
		angular: out.At(2, 0),
	}
}

// returns an array of power for each motors
// forwardPercent: -1 -> 1 percent of power in which you want to move laterally
//
//	note only x & y are relevant. y is forward back, x is lateral
//
// angularPercent: -1 -> 1 percent of power you want applied to move angularly
//
//	note only z is relevant here
func (cfg *Config) ComputePower(linear, angular r3.Vector) ([]float64, error) {
	goal := cfg.computeGoal(linear, angular)
	numMotrs := uint(len(cfg.Motors))
	opt, err := nlopt.NewNLopt(nlopt.GN_DIRECT, numMotrs)
	if err != nil {
		return nil, err
	}
	defer opt.Destroy()

	mins := []float64{}
	maxs := []float64{}

	for range cfg.Motors {
		mins = append(mins, -1)
		maxs = append(maxs, 1)
	}

	err = multierr.Combine(
		opt.SetLowerBounds(mins),
		opt.SetUpperBounds(maxs),

		opt.SetStopVal(.002),
		opt.SetMaxTime(.25),
	)
	if err != nil {
		return nil, err
	}

	myfunc := func(x, gradient []float64) float64 {
		total := cfg.ComputePowerOutput(x)
		return total.diff(goal)
	}

	err = opt.SetMinObjective(myfunc)
	if err != nil {
		return nil, err
	}
	powers, _, err := opt.Optimize(make([]float64, numMotrs))
	if err != nil {
		return nil, err
	}

	return powers, nil
}
