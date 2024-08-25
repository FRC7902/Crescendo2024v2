// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.commands.teleopCommands.intake.StopIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveIntakeComeBackLong extends SequentialCommandGroup {
  /** Creates a new DriveIntakeComeBack. */
  DriveAndIntake m_driveAndIntake;

  public DriveIntakeComeBackLong(DriveSubsystem drive, IntakeSubsystem intake, ArmSubsystem arm, double distance,
      boolean raiseArmToSpeaker) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_driveAndIntake = new DriveAndIntake(drive, intake, distance);
    addCommands(
        m_driveAndIntake.until(intake::hasNote),
        new StopIntake(intake).withTimeout(0.01),
        new ConditionalCommand(
            new SpeakerSetpoint(arm),
            new Level0Setpoint(arm),
            () -> raiseArmToSpeaker).withTimeout(0.01),
        new DriveToDistance(drive, m_driveAndIntake.getDistanceTravelled() * 0.8).withTimeout(3));
  }
}
