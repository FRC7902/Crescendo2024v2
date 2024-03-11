// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.drive.DriveRaw;
import frc.robot.commands.teleopCommands.drive.DriveToDistance;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.commands.teleopCommands.intake.IntakeNote;
import frc.robot.commands.teleopCommands.intake.StopIntake;
import frc.robot.commands.teleopCommands.shooter.SetSpeedSpeaker;
import frc.robot.commands.teleopCommands.shooter.StopShooter;
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
      new SpeakerSetpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new SetSpeedSpeaker(shooter).withTimeout(1),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new DriveAndIntake(drive, intake).withTimeout(2.5),
      new StopIntake(intake).withTimeout(0.01),
      new DriveToDistance(drive, -1.5).withTimeout(2.5),
      new SpeakerSetpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new SetSpeedSpeaker(shooter).withTimeout(1),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new DriveToDistance(drive, 3)
      );
  }
}
