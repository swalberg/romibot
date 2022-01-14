// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LidarCloud;

public class LidarWideScan extends CommandBase {
  private static final int STEP = 3; 
  private static final double MAX_RIGHT = 170;
  private static final double MAX_LEFT = 0;
  LidarCloud lidar;
  Timer timer = new Timer();
  double speed = 1;
  /** Creates a new LidarWideScan. */
  public LidarWideScan(LidarCloud lidar) {
    this.lidar = lidar;
    addRequirements(lidar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(0.1)) {
      double currentAngle = lidar.getAngle();
      if (currentAngle >= MAX_RIGHT) {
        speed = -STEP;
      }
      if (currentAngle <= MAX_LEFT) {
        speed = STEP;
      }

      lidar.setAngle(currentAngle + speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
