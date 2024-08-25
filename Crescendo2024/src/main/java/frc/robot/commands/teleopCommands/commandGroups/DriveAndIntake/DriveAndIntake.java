// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.commands.teleopCommands.intake.IntakeNote;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndIntake extends ParallelCommandGroup {
  /** Creates a new DriveAndIntake. */
  DriveToDistance m_driveToDistance;

  public DriveAndIntake(DriveSubsystem drive, IntakeSubsystem intake, double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_driveToDistance = new DriveToDistance(drive, distance);
    addCommands(
        m_driveToDistance,
        new IntakeNote(intake));
  }

  public double getDistanceTravelled() {
    return m_driveToDistance.getDistanceTravelled();
  }
}
