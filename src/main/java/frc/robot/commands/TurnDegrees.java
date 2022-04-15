// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TurnDegrees extends CommandBase {
  private static final double MAX_SPEED = 0.5;
  private final DriveTrain m_drive;
  private final double m_degrees;
  private final PIDController pid = new PIDController(Constants.kGainsDistance.kP, Constants.kGainsDistance.kI, Constants.kGainsDistance.kD);

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double degrees, DriveTrain drive) {
    m_degrees = degrees;
    m_drive = drive;
    pid.setTolerance(0.1, 0.1);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    /* Need to convert distance travelled to degrees. The Standard
       Romi Chassis found here, https://www.pololu.com/category/203/romi-chassis-kits,
       has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
       or 5.551 inches. We then take into consideration the width of the tires.
    */
    double inchPerDegree = Math.PI * 5.551 / 360;
    pid.setSetpoint(m_degrees * inchPerDegree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(pid.calculate(getAverageTurningDistance()), -MAX_SPEED, MAX_SPEED);
    m_drive.arcadeDrive(0, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to distance based on degree turn
    return pid.atSetpoint();
  }

  private double getAverageTurningDistance() {
    double leftDistance = Math.abs(m_drive.getLeftDistanceMeter());
    double rightDistance = Math.abs(m_drive.getRightDistanceMeter());
    double d = (leftDistance + rightDistance) / 2.0;
    return d;
  }
}
