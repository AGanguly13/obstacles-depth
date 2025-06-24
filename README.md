# Obstacles Depth Module

This module provides a service for reading depth images from a depth camera and detecting obstacles in the environment by projecting them onto a point cloud and applying a point
cloud clustering algorithm. 

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "min_points_in_plane": 500,
  "min_points_in_segment": 10,
  "max_dist_from_plane_mm": 100,
  "ground_angle_tolerance_degs": 30,
  "clustering_radius": 1,
  "clustering_strictness": 5,
  "camera_name": "camera-1"
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `min_points_in_plane` | float  | Required  | Minimum number of points in the plane |
| `min_points_in_segment` | float | Required  | Minimum number of points in the segment |
| `max_dist_from_plane_mm` | float | Required  | Maximum distance from the plane in mm |
| `ground_angle_tolerance_degs` | float | Required  | Ground angle tolerance in degrees |
| `clustering_radius` | float | Required  | Clustering radius in meters |
| `clustering_strictness` | float | Required  | Clustering strictness level |
| `camera_name` | string | Required  | Name of the camera to use |

### Example Camera Configuration

```json
{
  "name": "camera-2",
  "api": "rdk:component:camera",
  "model": "viam:camera:realsense",
  "attributes": {
    "width_px": 640,
    "height_px": 480,
    "little_endian_depth": false,
    "serial_number": "",
    "sensors": [
      "depth",
      "color"
    ]
  }
}
```

### Example Module Configuration

```json
{
  "name": "vision-1",
  "api": "rdk:service:vision",
  "model": "viam:vision:obstacles-depth",
  "attributes": {
    "min_points_in_plane": 500,
    "min_points_in_segment": 10,
    "max_dist_from_plane_mm": 100,
    "ground_angle_tolerance_degs": 30,
    "clustering_radius": 1,
    "clustering_strictness": 5,
    "camera_name": "camera-1"
  }
}
```
