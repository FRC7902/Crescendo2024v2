// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups.DriveAndArm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.teleopCommands.arm.incrementAngle;
import frc.robot.commands.teleopCommands.drive.DriveRaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndLiftArm extends ParallelCommandGroup {
  /** Creates a new DriveAndLiftArm. */
  public DriveAndLiftArm(DriveSubsystem drive, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveRaw(drive, -0.25),
        new incrementAngle(arm));
  }
}
