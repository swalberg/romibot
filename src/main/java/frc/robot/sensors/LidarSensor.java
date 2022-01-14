package frc.robot.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class LidarSensor {
    private SimDouble proximity;

    public LidarSensor() {
        SimDevice lidarSimDevice = SimDevice.create("Lidar:SimLidar");
        if (lidarSimDevice != null) {
            proximity = lidarSimDevice.createDouble("proximity", Direction.kOutput, -0.9);
        } else {
            System.out.println("Can't init the lidar");
        }
    }

    /**
     * Get the proximity in mm
     *
     * @return proximity to target in mm
     */
    public double getProximity() {
        if (proximity != null) {
            return proximity.get();
        }

        return 0.1;
    }

}
