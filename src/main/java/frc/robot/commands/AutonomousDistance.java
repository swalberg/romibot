// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  private static final double DISTANCE = 0.5;

  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(DISTANCE, drivetrain).withTimeout(2),
        new TurnDegrees(90, drivetrain).withTimeout(2),
        new DriveDistance(DISTANCE, drivetrain).withTimeout(2),
        new TurnDegrees(90, drivetrain).withTimeout(2),
        new DriveDistance(DISTANCE, drivetrain).withTimeout(2),
        new TurnDegrees(90, drivetrain).withTimeout(2),
        new DriveDistance(DISTANCE, drivetrain).withTimeout(2),
        new TurnDegrees(90, drivetrain).withTimeout(2),
        new PrintCommand("Auto routine done!")
        );

  }
}
