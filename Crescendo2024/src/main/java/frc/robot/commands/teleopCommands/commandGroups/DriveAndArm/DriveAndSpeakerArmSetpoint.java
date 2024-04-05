// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups.DriveAndArm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.DriveToDistance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndSpeakerArmSetpoint extends ParallelCommandGroup {
  /** Creates a new DriveAndSpeakerArmSetpoint. */
  public DriveAndSpeakerArmSetpoint(DriveSubsystem drive, ArmSubsystem arm, double distance) {
    // Add your commands in the addCommands() call, e.g
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToDistance(drive, distance).withTimeout(1.5),
      new SpeakerSetpoint(arm).until(arm:: atTargetPosition)
    );
  }
}
