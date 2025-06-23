//go:build !no_cgo

// Package obstaclesdepth uses an underlying depth camera to fulfill GetObjectPointClouds,
// projecting its depth map to a point cloud, an then applying a point cloud clustering algorithm
package obstaclesdepth

import (
	"context"
	"sort"
	"image"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/depthadapter"
	"go.viam.com/rdk/rimage/transform"
	// "go.viam.com/rdk/robot"
	vision "go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	// "go.viam.com/rdk/utils"
	vis "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/segmentation"
	"go.viam.com/rdk/vision/classification"
	objdet "go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"
	"go.viam.com/utils/rpc"
)

var Model = resource.NewModel("viam", "vision", "obstacles-depth")
var errUnimplemented = errors.New("obstacles depth service does not implement this method")

func init() {
	resource.RegisterService(vision.API, Model, resource.Registration[vision.Service, *ObsDepthConfig]{
		Constructor: func(
			ctx context.Context,
			deps resource.Dependencies,
			conf resource.Config,
			logger logging.Logger,
		) (vision.Service, error) {
			attrs, err := resource.NativeConfig[*ObsDepthConfig](conf)
			if err != nil {
				return nil, err
			}
			return registerObstaclesDepth(ctx, conf.ResourceName(), attrs, deps, logger)
		},
	})
}

// ObsDepthConfig specifies the parameters to be used for the obstacle depth service.
type ObsDepthConfig struct {
	resource.TriviallyValidateConfig
	MinPtsInPlane        int     `json:"min_points_in_plane"`
	MinPtsInSegment      int     `json:"min_points_in_segment"`
	MaxDistFromPlane     float64 `json:"max_dist_from_plane_mm"`
	ClusteringRadius     int     `json:"clustering_radius"`
	ClusteringStrictness float64 `json:"clustering_strictness"`
	AngleTolerance       float64 `json:"ground_angle_tolerance_degs"`
	DefaultCamera        string  `json:"camera_name"`
}

// obsDepth is the underlying struct actually used by the service.
type obsDepth struct {
	clusteringConf *segmentation.ErCCLConfig
	intrinsics     *transform.PinholeCameraIntrinsics
	deps           resource.Dependencies
	logger         logging.Logger
	name           resource.Name
	defaultCamera  camera.Camera
}

func (cfg *ObsDepthConfig) Validate(path string) ([]string, []string, error) {
	var reqDeps []string
	if cfg.DefaultCamera != "" {
		reqDeps = append(reqDeps, cfg.DefaultCamera)
	}
	return reqDeps, []string{}, nil
}

func registerObstaclesDepth(
	ctx context.Context,
	name resource.Name,
	conf *ObsDepthConfig,
	deps resource.Dependencies,
	logger logging.Logger,
) (vision.Service, error) {
	_, span := trace.StartSpan(ctx, "service::vision::registerObstacleDepth")
	defer span.End()
	if conf == nil {
		return nil, errors.New("config for obstacles_depth cannot be nil")
	}
	
	// build the clustering config
	cfg := &segmentation.ErCCLConfig{
		MinPtsInPlane:        conf.MinPtsInPlane,
		MinPtsInSegment:      conf.MinPtsInSegment,
		MaxDistFromPlane:     conf.MaxDistFromPlane,
		NormalVec:            r3.Vector{0, -1, 0},
		AngleTolerance:       conf.AngleTolerance,
		ClusteringRadius:     conf.ClusteringRadius,
		ClusteringStrictness: conf.ClusteringStrictness,
	}
	err := cfg.CheckValid()
	if err != nil {
		return nil, errors.Wrap(err, "error building clustering config for obstacles_depth")
	}

	// Get camera dependency if specified
	var defaultCam camera.Camera
	if conf.DefaultCamera != "" {
		defaultCam, err = camera.FromDependencies(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
	}

	myObsDep := &obsDepth{
		clusteringConf: cfg,
		deps:           deps,
		logger:         logger,
		name:           name,
		defaultCamera:  defaultCam,
	}

	segmenter := myObsDep.buildObsDepth(logger)
	return vision.NewService(name, nil, nil, nil, nil, segmenter, conf.DefaultCamera)
}

// BuildObsDepth will check for intrinsics and determine how to build based on that.
func (o *obsDepth) buildObsDepth(logger logging.Logger) func(
	ctx context.Context, src camera.Camera) ([]*vis.Object, error) {
	return func(ctx context.Context, src camera.Camera) ([]*vis.Object, error) {
		props, err := src.Properties(ctx)
		if err != nil {
			logger.CWarnw(ctx, "could not find camera properties. obstacles depth started without camera's intrinsic parameters", "error", err)
			return o.obsDepthNoIntrinsics(ctx, src)
		}
		if props.IntrinsicParams == nil {
			logger.CWarn(ctx, "obstacles depth started but camera did not have intrinsic parameters")
			return o.obsDepthNoIntrinsics(ctx, src)
		}
		o.intrinsics = props.IntrinsicParams
		return o.obsDepthWithIntrinsics(ctx, src)
	}
}

// buildObsDepthNoIntrinsics will return the median depth in the depth map as a Geometry point.
func (o *obsDepth) obsDepthNoIntrinsics(ctx context.Context, src camera.Camera) ([]*vis.Object, error) {
	img, err := camera.DecodeImageFromCamera(ctx, "", nil, src)
	if err != nil {
		return nil, errors.Errorf("could not get image from %s", src)
	}

	dm, err := rimage.ConvertImageToDepthMap(ctx, img)
	if err != nil {
		return nil, errors.New("could not convert image to depth map")
	}
	depData := dm.Data()
	if len(depData) == 0 {
		return nil, errors.New("could not get info from depth map")
	}
	// Sort the depth data [smallest...largest]
	sort.Slice(depData, func(i, j int) bool {
		return depData[i] < depData[j]
	})
	med := int(0.5 * float64(len(depData)))
	pt := spatialmath.NewPoint(r3.Vector{X: 0, Y: 0, Z: float64(depData[med])}, "")
	toReturn := make([]*vis.Object, 1)
	toReturn[0] = &vis.Object{Geometry: pt}
	return toReturn, nil
}

// buildObsDepthWithIntrinsics will use the methodology in Manduchi et al. to find obstacle points
// before clustering and projecting those points into 3D obstacles.
func (o *obsDepth) obsDepthWithIntrinsics(ctx context.Context, src camera.Camera) ([]*vis.Object, error) {
	// Check if we have intrinsics here. If not, don't even try
	if o.intrinsics == nil {
		return nil, errors.New("tried to build obstacles depth with intrinsics but no instrinsics found")
	}
	img, err := camera.DecodeImageFromCamera(ctx, "", nil, src)
	if err != nil {
		return nil, errors.Errorf("could not get image from %s", src)
	}
	dm, err := rimage.ConvertImageToDepthMap(ctx, img)
	if err != nil {
		return nil, errors.New("could not convert image to depth map")
	}
	cloud := depthadapter.ToPointCloud(dm, o.intrinsics)
	return segmentation.ApplyERCCLToPointCloud(ctx, cloud, o.clusteringConf)
}

func (s *obsDepth) Name() resource.Name {
    return s.name
}

func (s *obsDepth) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
    return nil
}

func (s *obsDepth) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (vision.Service, error) {
	return nil, errUnimplemented
}
func (s *obsDepth) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}
func (s *obsDepth) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}
func (s *obsDepth) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}
func (s *obsDepth) Classifications(ctx context.Context, img image.Image, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}

func (s *obsDepth) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
	var cam camera.Camera
	var err error
	
	if cameraName != "" {
		cam, err = camera.FromDependencies(s.deps, cameraName)
		if err != nil {
			return nil, err
		}
	} else if s.defaultCamera != nil {
		cam = s.defaultCamera
	} else {
		return nil, errors.New("no camera specified")
	}
	
	// Use your existing segmenter logic
	segmenter := s.buildObsDepth(s.logger)
	return segmenter(ctx, cam)
}

func (s *obsDepth) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	return &vision.Properties{
        ClassificationSupported: false,
        DetectionSupported:      false,
        ObjectPCDsSupported:     true,
    }, nil
}

func (s *obsDepth) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	return viscapture.VisCapture{}, errUnimplemented
}

func (s *obsDepth) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, errUnimplemented
}

func (s *obsDepth) Close(context.Context) error {
	// Put close code here
	// s.cancelFunc()
	return nil
}
