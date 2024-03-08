// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake;
import frc.robot.commands.teleopCommands.drive.DriveRaw;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.commands.teleopCommands.shooter.ShootSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAutoSimple extends SequentialCommandGroup {
  /** Creates a new TwoNoteAutoSimple. */
  public TwoNoteAutoSimple(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
      new SpeakerSetpoint(arm).withTimeout(1),
      new ShootSpeaker(shooter).withTimeout(1),
      new FeedNote(intake).withTimeout(1),
      new Level0Setpoint(arm).withTimeout(1),
      new DriveAndIntake(drive, intake).withTimeout(1),
      new DriveRaw(drive, AutoConstants.autoDriveSpeed).withTimeout(1),
      new SpeakerSetpoint(arm).withTimeout(1),
      new ShootSpeaker(shooter).withTimeout(1),
      new FeedNote(intake).withTimeout(1),
      new Level0Setpoint(arm)
      );
  }
}