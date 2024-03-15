// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.drive.DriveToDistance;
import frc.robot.commands.teleopCommands.drive.ScanField;
import frc.robot.commands.teleopCommands.drive.SetStartingPosition;
import frc.robot.commands.teleopCommands.drive.TurnToAngle;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.commands.teleopCommands.intake.StopIntake;
import frc.robot.commands.teleopCommands.shooter.SetSpeedSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteAutoMiddle extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAutoMiddle. */
  public ThreeNoteAutoMiddle(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(  
      //new ScanField(drive).withTimeout(3),    
      new SetStartingPosition(drive, 0, 1.5, 5.5),
      new SpeakerSetpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new SetSpeedSpeaker(shooter).until(shooter::atTargetSpeed).withTimeout(2),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).withTimeout(2),
      new DriveAndIntake(drive, intake, 1.5).withTimeout(5),
      new StopIntake(intake).withTimeout(0.01),
      new DriveToDistance(drive, -1.6).withTimeout(5),
      new SpeakerSetpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new SetSpeedSpeaker(shooter).until(shooter::atTargetSpeed),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new DriveToDistance(drive, 1.75),
      new TurnToAngle(drive, 90, true),
      new DriveAndIntake(drive, intake, 1).withTimeout(2.5),
      new StopIntake(intake).withTimeout(0.01),
      new DriveToDistance(drive, -1).withTimeout(2),
      new TurnToAngle(drive, -90, true),
      new DriveToDistance(drive, -1.75).withTimeout(2.5),
      new SpeakerSetpoint(arm).until(arm::atTargetPosition).withTimeout(1),
      new SetSpeedSpeaker(shooter).until(shooter::atTargetSpeed),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).until(arm::atTargetPosition).withTimeout(1)
      );

  }
}
