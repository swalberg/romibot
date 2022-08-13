package frc.robot.sensors;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
     private PhotonCamera camera;

    public Camera() {
        camera = new PhotonCamera("picam");
    }

    public boolean hasTarget() {
        var result = camera.getLatestResult();

        return result.hasTargets();
    }

    /**
     * Figures out the angle to the target
     * @return angle to target, in degrees
     */
    public double angleToTarget() {
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            return result.getBestTarget().getYaw();
        } else {
            return 0;
        }
    }

    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("HasTargets", camera.getLatestResult().hasTargets());
        //SmartDashboard.putNumber("AngleToTarget", angleToTarget());
    }
}
