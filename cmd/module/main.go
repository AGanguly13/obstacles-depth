package main

import (
	"context"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"

	// Import your obstacles depth package
	_ "obstaclesdepth/obstacles-depth"
)

func main() {
	utils.ContextualMain(mainWithArgs, logging.NewDebugLogger("obstacles-depth"))
}

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) error {
	obstacleModule, err := module.NewModuleFromArgs(ctx)
	if err != nil {
		return err
	}

	err = obstacleModule.Start(ctx)
	defer obstacleModule.Close(ctx)
	if err != nil {
		return err
	}

	<-ctx.Done()
	return nil
}
