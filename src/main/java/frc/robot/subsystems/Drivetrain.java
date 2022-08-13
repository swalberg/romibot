// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;

public class DriveTrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = 70.0 / 1000.0; // 70 mm
  private static final double kTrackWidth = 141.0 / 1000.0;
  PIDController pid = new PIDController(0.35, 0.05, 0.02);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d(0.0, 0.0, new Rotation2d()));

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private Pose2d m_pose;

  private Rotation2d gyroAngle = new Rotation2d();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
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
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    /*
    if (zaxisRotate == 0) {
      m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
      double output = pid.calculate(m_accelerometer.getZ(), 0);
      SmartDashboard.putNumber("straightpid", output);
      m_diffDrive.arcadeDrive(xaxisSpeed, pid.calculate(m_accelerometer.getZ(), 0));
    } else { */
      m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    //}
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
   * @return distance, in meters
   */
  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(- getGyroAngleZ());
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    //SmartDashboard.putNumber("TurnRate", chassisSpeeds.omegaRadiansPerSecond);

    gyroAngle = gyroAngle.plus(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond / 50.0)); // 50 time slices per second
    //SmartDashboard.putNumber("AngleKinematics", gyroAngle.getDegrees());
    //SmartDashboard.putString("AngleGyro", getAngle().toString());
    
    m_pose = m_odometry.update(getAngle(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    SmartDashboard.putString("Pose", m_pose.toString());
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
