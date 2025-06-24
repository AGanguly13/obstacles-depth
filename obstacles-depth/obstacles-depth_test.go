package obstaclesdepth

import (
	"context"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/vision/viscapture"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/rimage"

	"go.viam.com/test"
)

func setupTest(t *testing.T) (camera.Camera, *ObsDepthConfig, context.Context, vision.Service) {
	fake_logger := logging.NewTestLogger(t)
	fake_context := context.Background()
	fake_dependencies := make(resource.Dependencies)

	fake_camera := &inject.Camera{}
    fake_camera.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
        return camera.Properties{
            SupportsPCD: true,
            IntrinsicParams: &transform.PinholeCameraIntrinsics{
				Width: 10,
				Height: 10,
			},
        }, nil
    }

	fake_camera.ImageFunc = func(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
		width, height := 10, 10
		depthMap := rimage.NewEmptyDepthMap(width, height)
		
		// Fill the depth map with some test data
		for y := 0; y < height; y++ {
			for x := 0; x < width; x++ {
				depthMap.Set(x, y, rimage.Depth(1000 + x*10 + y*10)) // Example depth values
			}
		}
		
		// Return the depth map as raw depth format
		img, err := rimage.EncodeImage(ctx, depthMap, mimeType)
		if err != nil {
			return nil, camera.ImageMetadata{}, err
		}
		
		return img, camera.ImageMetadata{MimeType: mimeType}, nil
	}

	fake_conf := &ObsDepthConfig{
		MinPtsInPlane:        500,
		MinPtsInSegment:      10,
		MaxDistFromPlane:     100,
		ClusteringRadius:     1,
		ClusteringStrictness: 5,
		AngleTolerance:       30,
		DefaultCamera:        "fake-camera",
	}

	fake_vision_service, err := registerObstaclesDepth(
		fake_context,
		resource.NewName(vision.API, "fake-vision-service"),
		fake_conf,
		fake_dependencies,
		fake_logger,
	)
	test.That(t, err, test.ShouldBeNil)

	return fake_camera, fake_conf, fake_context, fake_vision_service
}

func TestValidate(t *testing.T) {
	_, conf, _, _ := setupTest(t)

	// Case 1: Expected config
	required_dependencies, optional_dependencies, err := conf.Validate("test")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, required_dependencies, test.ShouldResemble, []string{"fake-camera"})
	test.That(t, optional_dependencies, test.ShouldBeNil)

	// Case 2: Empty config
	conf_empty := &ObsDepthConfig{}
	required_dependencies, optional_dependencies, err = conf_empty.Validate("test")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, required_dependencies, test.ShouldBeNil)
	test.That(t, optional_dependencies, test.ShouldBeNil)
}

func TestGetProperties(t *testing.T) {
	_, _, fake_context, fake_vision_service := setupTest(t)
	fake_properties, err := fake_vision_service.GetProperties(fake_context, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, fake_properties.ClassificationSupported, test.ShouldBeFalse)
	test.That(t, fake_properties.DetectionSupported, test.ShouldBeFalse)
	test.That(t, fake_properties.ObjectPCDsSupported, test.ShouldBeTrue)
}

func TestCaptureAllFromCamera(t *testing.T) {
	_, _, fake_context, fake_vision_service := setupTest(t)
	defaultCaptureOptions := viscapture.CaptureOptions{
		ReturnImage:  true,
		ReturnObject: true,
	}

	test_capture_results, err := fake_vision_service.CaptureAllFromCamera(
		fake_context,
		"fake-camera",
		defaultCaptureOptions,
		nil,
	)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, test_capture_results.Image, test.ShouldNotBeNil)
	test.That(t, test_capture_results.Objects, test.ShouldNotBeNil)
	test.That(t, test_capture_results.Detections, test.ShouldBeEmpty)
	test.That(t, test_capture_results.Classifications, test.ShouldBeEmpty)
}

func TestGetObjectPointCloud(t *testing.T) {
	_, _, fake_context, fake_vision_service := setupTest(t)

	test_objects, err := fake_vision_service.GetObjectPointClouds(
		fake_context,
		"fake-camera",
		nil,
	)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, test_objects, test.ShouldNotBeNil)
}
