package main

import (
	"context"

	"github.com/edaniels/golog"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/module"

	"github.com/erh/viamboatbase"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}
func realMain() error {

	ctx := context.Background()
	logger := golog.NewDevelopmentLogger("client")

	myMod, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	err = myMod.AddModelFromRegistry(ctx, base.API, viamboatbase.Model)
	if err != nil {
		return err
	}

	err = myMod.Start(ctx)
	defer myMod.Close(ctx)
	if err != nil {
		return err
	}
	<-ctx.Done()
	return nil
}
