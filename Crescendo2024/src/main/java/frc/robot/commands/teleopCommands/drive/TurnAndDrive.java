// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.teleopCommands.drive.TurnToAngle;
import frc.robot.FireBirdsUtils;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAndDrive extends SequentialCommandGroup {
  private double targetY;
  private double targetX;
  private DriveSubsystem m_drive;
  private final FireBirdsUtils util = new FireBirdsUtils();

  /** Creates a new TurnAndDrive. */
  public TurnAndDrive(DriveSubsystem drive, double Y, double X) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    targetY = Y;
    targetX = X;
    double target = util.TurnToPoint(Y, X);
    addCommands(
        new TurnToAngle(m_drive, target, true));
        new DriveToDistance(m_drive, util.FindDistance(targetY, targetX));
  }
}
