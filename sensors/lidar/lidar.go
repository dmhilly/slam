// Package lidar implements the Lidar sensor.
// Warning: The code in this package might be transient. Stability is not guaranteed.
package lidar

import (
	"context"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"

	"go.viam.com/slam/sensors/utils"
)

// Lidar represents a LIDAR sensor.
type Lidar struct {
	Name  string
	lidar camera.Camera
}

// New creates a new Lidar sensor based on the sensor definition and the service config.
func New(deps resource.Dependencies, sensors []string, sensorIndex int) (Lidar, error) {
	name, err := utils.GetName(sensors, sensorIndex)
	if err != nil {
		return Lidar{}, err
	}

	newLidar, err := camera.FromDependencies(deps, name)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting lidar camera %v for slam service", name)
	}

	return Lidar{
		Name:  name,
		lidar: newLidar,
	}, nil
}

// GetData returns data from the lidar sensor.
func (lidar Lidar) GetData(ctx context.Context) (pointcloud.PointCloud, time.Time, error) {
	pcd, err := lidar.lidar.NextPointCloud(ctx)
	if err != nil {
		return nil, time.Time{}, err
	}

	// If the camera provides timestamp information, extract the time. Otherwise, return the
	// current time.
	timeRec := time.Now()
	if pcdSourceWithTimestamps, ok := lidar.lidar.(camera.PointCloudSourceWithTimestamps); ok {
		_, timeRec, err = pcdSourceWithTimestamps.NextPointCloudTimestamps(ctx)
		if err != nil {
			return nil, time.Time{}, err
		}
	}

	return pcd, timeRec, nil
}
