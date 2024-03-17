// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

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
import frc.robot.commands.teleopCommands.shooter.SetSpeedSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteMiddleFaster extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAutoMiddle. */
  public ThreeNoteMiddleFaster(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter, int mirror) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(  
      new SetStartingPosition(drive, 0, 1.5, 5.5),
      new SpeakerArmAndShooter(arm, shooter).withTimeout(3),
      new FeedNote(intake).withTimeout(0.5),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).withTimeout(2),
      new DriveIntakeComeBack(drive, intake, arm, shooter, 1.5, true, true).until(shooter::atTargetSpeed),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new TurnToAngle(drive, mirror * (-45), false).withTimeout(2),
      new DriveIntakeComeBack(drive, intake, arm, shooter, 2.12, true, true).until(shooter::atTargetSpeed),
      new TurnToAngle(drive, 0, false).withTimeout(2),
      new FeedNote(intake).withTimeout(0.01),
      new Level0Setpoint(arm)
      );

  }
}
