// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.LidarSensor;

public class LidarCloud extends SubsystemBase {
  private static final double SCAN_PERIOD = 0.2;
  LidarSensor distance = new LidarSensor();
  Servo swivel = new Servo(Constants.LIDAR_SWIVEL_PORT);
  Timer timer = new Timer();
  double currentAngle = 0;

  /** Creates a new LidarCloud. */
  public LidarCloud() {
    timer.start();
    swivel.setAngle(currentAngle);
  }

  public void setAngle(double angle) {
    currentAngle = angle;
  }

  public double getAngle() {
    return currentAngle;
  }

  @Override
  public void periodic() {
    if (timer.advanceIfElapsed(SCAN_PERIOD)) {
    swivel.setAngle(currentAngle);
      SmartDashboard.putNumber("Lidar-prox", distance.getProximity());
      SmartDashboard.putNumber("Lidar-heading", currentAngle);
    }
  }
}
