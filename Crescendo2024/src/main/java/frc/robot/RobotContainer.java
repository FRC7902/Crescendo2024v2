// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.teleopCommands.drive.DriveToDistance;
import frc.robot.commands.teleopCommands.drive.TurnToAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(camera);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
 private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);

SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      m_driveSubsystem.setDefaultCommand(
        new RunCommand(
          () -> m_driveSubsystem.driveArcade(
             m_driverStick.getRawAxis(Constants.IOConstants.kLY),
             m_driverStick.getRawAxis(Constants.IOConstants.kLX)),
             m_driveSubsystem));

    // new JoystickButton(m_driverStick, IOConstants.kA).onTrue(new TurnToAngle(m_driveSubsystem, 30, true));
    // new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new TurnToAngle(m_driveSubsystem, 0, false));
    // new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new DriveToDistance(m_driveSubsystem, 1));
    new JoystickButton(m_driverStick, IOConstants.kA).onTrue(new PathPlannerAuto("ScoreAmp"));
    new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new PathPlannerAuto("LoadPiece"));
    new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new SetArmPosition(m_armSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return new PathPlannerAuto("New Auto");
    //return m_chooser.getSelected();
  }
}
