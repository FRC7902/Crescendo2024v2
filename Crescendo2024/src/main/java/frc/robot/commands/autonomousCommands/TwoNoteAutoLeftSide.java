// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.commandGroups.ArmAndShooter.SpeakerArmAndShooter;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake.DriveIntakeComeBackLong;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.TurnToAngle;
import frc.robot.commands.teleopCommands.drive.odometry.SetStartingPosition;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.commands.teleopCommands.intake.IntakeNote;
import frc.robot.commands.teleopCommands.intake.OuttakeNote;
import frc.robot.commands.teleopCommands.shooter.SetSpeedSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAutoLeftSide extends SequentialCommandGroup {
  /** Creates a new TwoNoteAutoSimple. */
  public TwoNoteAutoLeftSide(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm, int mirror, boolean taxi) {
    addCommands(
      new SpeakerArmAndShooter(arm, shooter).withTimeout(3),
      new FeedNote(intake).withTimeout(0.5),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).withTimeout(2),
      new DriveToDistance(drive, 0.4096), //24 * 2.54 * 0.01
      new TurnToAngle(drive, mirror * (65), true).withTimeout(2),
      new DriveIntakeComeBackLong(drive, intake, arm, 1.5, true),
      new TurnToAngle(drive, mirror * -65, true),
      new DriveToDistance(drive, (-24) * 2.54 * 0.01).withTimeout(2),
      new SetSpeedSpeaker(shooter).until(shooter::atTargetSpeed),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new TurnToAngle(drive, mirror * (65), true).withTimeout(2),
      new ConditionalCommand(
        new DriveToDistance(drive, 1), 
        new InstantCommand(), 
        () -> taxi)
      );
  }
}
