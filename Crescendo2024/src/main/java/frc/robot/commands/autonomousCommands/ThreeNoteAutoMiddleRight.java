// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.commandGroups.ArmAndShooter.SpeakerArmAndShooter;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake.DriveIntakeComeBack;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndShooter.DriveAndRevSpeaker;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.TurnToAngle;
import frc.robot.commands.teleopCommands.drive.odometry.SetStartingPosition;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteAutoMiddleRight extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAutoMiddle. */
  public ThreeNoteAutoMiddleRight(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter, int mirror, boolean taxi) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(  
      new SetStartingPosition(drive, 0, 1.5, 5.5),
      // new ConditionalCommand(new FeedNote(intake).withTimeout(0.3), new InstantCommand(), intake::hasNote),
      new SpeakerArmAndShooter(arm, shooter).withTimeout(3),
      new FeedNote(intake).withTimeout(0.5),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(2),
      new DriveIntakeComeBack(drive, intake, arm, shooter, 1.5, true, true),
      new FeedNote(intake).withTimeout(0.5),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new DriveToDistance(drive, 1.6).withTimeout(3),
      new TurnToAngle(drive, mirror * (90), false).withTimeout(3),
      new DriveIntakeComeBack(drive, intake, arm, shooter, 1, false, false),
      new TurnToAngle(drive, 0, false).withTimeout(3),
      new SpeakerSetpoint(arm).withTimeout(0.01),
      new DriveAndRevSpeaker(drive, shooter, -1.6).withTimeout(1.5),
      new FeedNote(intake).withTimeout(0.5),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new ConditionalCommand(
        new DriveToDistance(drive, 1), 
        new InstantCommand(), 
        () -> taxi)
      );

  }
}
