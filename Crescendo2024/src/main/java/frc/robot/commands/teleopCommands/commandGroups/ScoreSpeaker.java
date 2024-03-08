// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.commands.teleopCommands.shooter.ShootSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreSpeaker extends SequentialCommandGroup {
  public ScoreSpeaker(ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
      new SpeakerSetpoint(arm).withTimeout(1),
      new ShootSpeaker(shooter).withTimeout(1),
      new FeedNote(intake).withTimeout(1),
      new Level0Setpoint(arm)
    );
  }
}
