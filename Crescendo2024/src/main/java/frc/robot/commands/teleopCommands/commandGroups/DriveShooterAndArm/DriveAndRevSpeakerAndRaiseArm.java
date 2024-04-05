// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups.DriveShooterAndArm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.commandGroups.DriveAndIntake.DriveAndIntake;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.commands.teleopCommands.shooter.SetSpeedSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndRevSpeakerAndRaiseArm extends ParallelCommandGroup {
  DriveToDistance m_DriveToDistance;
  /** Creates a new DriveAndRevSpeakerAndRaiseArm. */
  public DriveAndRevSpeakerAndRaiseArm(DriveSubsystem drive, ShooterSubsystem shooter, ArmSubsystem arm, double distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_DriveToDistance = new DriveToDistance(drive, distance);
    addCommands(
      new DriveToDistance(drive, distance).until(() -> 0.1 > Math.abs(m_DriveToDistance.getDistanceTravelled() - distance)).withTimeout(5),
      new SetSpeedSpeaker(shooter).until(shooter::atTargetSpeed),
      new SpeakerSetpoint(arm).until(arm::atTargetPosition)
    );
  }
}
