// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int LIDAR_SWIVEL_PORT = 2;
    public final class DriveTrain {
    }

    public static final class DriveConstants {
        public static final double ksVolts = 0.929;
        public static final double kvVoltSecondsPerMeter = 6.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;
    
        public static final double kPDriveVel = 1.0;
        public static final double kIDriveVel = 0.0;
        public static final double kDDriveVel = 0.0;
    
        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

        public static final double maxVelocityMetersPerSecond = 0.6;
      }

    //                                                   P     I     D    F
    public final static Gains kGainsDistance = new Gains(0.35, 0.05, 0.05, 0.0, 0, 0.0);
    public final static Gains kGainsVelocity = new Gains(1.0, 0.05, 0.05, 0.0, 0, 0.0);
}