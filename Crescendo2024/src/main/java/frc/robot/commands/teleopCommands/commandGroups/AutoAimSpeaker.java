// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.FireBirdsUtils;
import frc.robot.commands.teleopCommands.arm.SetAutoAimStatus;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.TurnToAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimSpeaker extends ParallelCommandGroup {
  private final FireBirdsUtils util = new FireBirdsUtils();

  public AutoAimSpeaker(ArmSubsystem arm, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetAutoAimStatus(arm),
        new TurnToAngle(drive, util.TurnToPoint(0, 0), true));
  }
}
