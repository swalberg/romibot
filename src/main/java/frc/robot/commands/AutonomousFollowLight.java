// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.DriveTrain;

public class AutonomousFollowLight extends CommandBase {
  DriveTrain driveTrain;
  Camera camera;
  final double ANGULAR_P = 0.02;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new AutonomousFollowLight. */
  public AutonomousFollowLight(DriveTrain driveTrain, Camera camera) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putData("TurnPID", turnController);
    if (camera.hasTarget()) {
      double angle = camera.angleToTarget();
      driveTrain.arcadeDrive(0, -turnController.calculate(angle));
    } else {
      driveTrain.arcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
