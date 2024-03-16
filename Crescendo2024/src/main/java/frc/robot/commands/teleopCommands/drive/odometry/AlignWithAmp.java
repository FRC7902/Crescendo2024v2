// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.drive.odometry;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleopCommands.drive.encoder_gyro.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithAmp extends SequentialCommandGroup {
  /** Creates a new AlignWithAmp. */
  public AlignWithAmp(DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToCoordinateX(drive, 2),
      new TurnToAngle(drive, 90, false),
      new DriveToCoordinateY(drive, 0.5)
    );
  }
}
