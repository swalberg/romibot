// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.RomiGyro;

public class DriveTrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 70.0 / 1000.0; // 70 mm

  private final PIDController leftPID = new PIDController(Constants.kGainsVelocity.kP, Constants.kGainsVelocity.kI,
      Constants.kGainsVelocity.kD);
  private final PIDController rightPID = new PIDController(Constants.kGainsVelocity.kP, Constants.kGainsVelocity.kI,
      Constants.kGainsVelocity.kD);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  private Pose2d m_pose;
  private final Field2d m_field = new Field2d();

  /*
   * Here we use DifferentialDrivePoseEstimator so that we can fuse odometry
   * readings. The
   * numbers used below are robot specific, and should be tuned.
   */
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      getAngle(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01), // State measurement standard deviations. X,
                                                                          // Y, theta.
      VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)), // Local measurement standard deviations. Left encoder,
                                                              // right encoder, gyro.
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // Vision measurement standard deviations. X, Y, and
                                                              // theta.

  /** Creates a new Drivetrain. */
  public DriveTrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();
    SmartDashboard.putData("Field", m_field);

  }

  /**
   * Drives the robot at a given speed and rotation rate
   * 
   * @param xaxisSpeed  forward/backward speed in m/s
   * @param zaxisRotate rotation rate in radians/sec
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    var m_deadband = 0.05;
    xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);
    zRotation = MathUtil.applyDeadband(zRotation, m_deadband);
    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
    zRotation = Math.copySign(zRotation * zRotation, zRotation);


    var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRotation));
    setSpeeds(wheelSpeeds);
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  private void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftPID.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPID.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftMotor.setVoltage(leftOutput + leftFeedforward);
    m_rightMotor.setVoltage(rightOutput + rightFeedforward);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
  }

  public double getRightVelocity() {
    return m_rightEncoder.getRate();
  }

  /**
   * Calculates the distance driven, in meters, using the average of both wheels
   * 
   * @return distance, in meters
   */
  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-m_gyro.getAngleZ());
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {

    m_pose = m_poseEstimator.update(
      getAngle(),
      new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate()),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance());


    m_field.setRobotPose(m_pose);
    SmartDashboard.putString("Pose", getPose().toString());
    SmartDashboard.putNumber("LeftDistance", getLeftDistanceMeter());
    SmartDashboard.putNumber("RightDistance", getRightDistanceMeter());
    SmartDashboard.putNumber("LeftVelocity", getLeftVelocity());
    SmartDashboard.putNumber("RightVelocity", getRightVelocity());
  }

  /** Returns the pose of the robot */
  public Pose2d getPose() {
    return m_pose;
  }

  public void setPose(Pose2d pose) {
    m_pose = pose;
  }

  /** For auto - drives the controllers at a given voltage */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("LeftVolts", leftVolts);
    SmartDashboard.putNumber("RightVolts", rightVolts);
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
  }
}
