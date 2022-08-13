// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class FollowTrajectory extends CommandBase {
  private static final double DT = 0.02;
  /** Creates a new FollowTrajectory. */
  PIDController leftPid, rightPid;

  LTVUnicycleController l;
  DriveTrain driveTrain;
  Trajectory trajectory;

  Timer time = new Timer();

  public FollowTrajectory(DriveTrain d, Trajectory t) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
    driveTrain = d;
    trajectory = t;
    l = new LTVUnicycleController(DT);
    leftPid = new PIDController(Constants.kGainsVelocity.kP, 0, 0);
    rightPid = new PIDController(Constants.kGainsVelocity.kP, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    double d = trajectory.getTotalTimeSeconds();
    System.out.printf("Trajectory is %f\n", d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State s = trajectory.sample(time.get());
    SmartDashboard.putString("State", s.toString());

    ChassisSpeeds c = l.calculate(driveTrain.getPose(), s);
    DifferentialDriveWheelSpeeds speeds = DriveConstants.kDriveKinematics.toWheelSpeeds(c);

    driveTrain.tankDriveVolts(leftPid.calculate(driveTrain.getLeftVelocity(), speeds.leftMetersPerSecond),
        rightPid.calculate(driveTrain.getRightVelocity(), speeds.rightMetersPerSecond));

    SmartDashboard.putNumber("leftPIDerror", leftPid.getPositionError());
    SmartDashboard.putNumber("leftPIDsetpoint", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("rightPIDerror", rightPid.getPositionError());
    SmartDashboard.putNumber("rightPIDsetpoint", speeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get() >= trajectory.getTotalTimeSeconds();
  }
}