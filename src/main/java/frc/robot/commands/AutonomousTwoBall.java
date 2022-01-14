// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTwoBall extends SequentialCommandGroup {
  /** Creates a new AutonomousTwoBall. */
  public AutonomousTwoBall(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveDistance(20, drivetrain).withTimeout(2),
        new TurnDegrees(180, drivetrain).withTimeout(2),
        new DriveDistance(20, drivetrain).withTimeout(2),
        new PrintCommand("Auto routine done!")
    );
  }
}
