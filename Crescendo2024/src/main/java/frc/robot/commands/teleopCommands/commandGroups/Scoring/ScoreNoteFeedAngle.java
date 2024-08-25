// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups.Scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.commandGroups.ArmAndShooter.FeedArmAndShooter;
import frc.robot.commands.teleopCommands.intake.FeedNote;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreNoteFeedAngle extends SequentialCommandGroup {
  /** Creates a new ScoreNoteFeedAngle. */
  public ScoreNoteFeedAngle(ShooterSubsystem shooter, ArmSubsystem arm, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new FeedArmAndShooter(arm, shooter).withTimeout(3),
        new FeedNote(intake));
  }
}
