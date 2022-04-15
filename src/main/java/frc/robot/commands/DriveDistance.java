// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private static final double MAX_SPEED = 0.7;
  double meters;
  DriveTrain drive;
  private final ProfiledPIDController pid = new ProfiledPIDController(Constants.kGainsDistance.kP,
   Constants.kGainsDistance.kI,
    Constants.kGainsDistance.kD,
    new TrapezoidProfile.Constraints(100.0, 200.0),
    0.02);

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a desired distance at
   * a desired speed.
   *
   * @param meters The number of meters the robot will drive
   * @param drive The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double meters, DriveTrain drive) {
    this.meters = meters;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(0.1, 0.05);
    drive.arcadeDrive(0, 0);
    drive.resetEncoders();
  }

  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  @Override
  public void execute() {
    //double output = MathUtil.clamp(pid.calculate(drive.getAverageDistanceMeter()), -MAX_SPEED, MAX_SPEED);
    double output = pid.calculate(drive.getAverageDistanceMeter(), meters);
    SmartDashboard.putNumber("ErrorD", pid.getPositionError());
    SmartDashboard.putNumber("ErrorV", pid.getVelocityError());
    SmartDashboard.putNumber("ProfileOutput", output);
    drive.arcadeDrive(output, 0);
  }

  @Override
  public boolean isFinished() {
    return pid.atGoal();
  }
}
