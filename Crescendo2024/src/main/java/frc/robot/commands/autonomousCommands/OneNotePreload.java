// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.commands.teleopCommands.shooter.SetSpeedSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNotePreload extends SequentialCommandGroup {
  public boolean m_taxi = false;

  /** Creates a new DriveOut. */
  public OneNotePreload(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
      new SpeakerSetpoint(arm).withTimeout(1),
      new SetSpeedSpeaker(shooter).withTimeout(1),
      new FeedNote(intake).withTimeout(1),
      new StopIntakeAndShooter(intake, shooter).withTimeout(0.01),
      new Level0Setpoint(arm).withTimeout(5),
      new WaitCommand(5),
      new ConditionalCommand(
        new DriveToDistance(drive, 1.5), 
        new InstantCommand(), 
        () -> m_taxi)
    );
  }

  public void setTaxi(boolean setting){
    m_taxi = setting;
  }
}
