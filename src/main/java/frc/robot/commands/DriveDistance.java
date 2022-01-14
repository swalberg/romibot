// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private static final double MAX_SPEED = 0.7;
  double meters;
  Drivetrain drive;
  private final PIDController pid = new PIDController(Constants.kGainsDistance.kP, Constants.kGainsDistance.kI, Constants.kGainsDistance.kD);

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param meters The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double meters, Drivetrain drive) {
    this.meters = meters;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0.1, 0.05);
    pid.setSetpoint(meters);
    drive.arcadeDrive(0, 0);
    drive.resetEncoders();
  }

  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  @Override
  public void execute() {
    double output = MathUtil.clamp(pid.calculate(drive.getAverageDistanceMeter()), -MAX_SPEED, MAX_SPEED);
    drive.arcadeDrive(output, 0);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
