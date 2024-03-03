// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StopIntake;
import frc.robot.commands.FeedNote;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootNoteAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.teleopCommands.arm.AmpSetpoint;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.drive.DriveRaw;
import frc.robot.commands.teleopCommands.AmpSetpoint;
import frc.robot.commands.teleopCommands.Level0Setpoint;
import frc.robot.commands.teleopCommands.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.climb.ClimbDown;
import frc.robot.commands.teleopCommands.climb.ClimbUp;
import frc.robot.commands.teleopCommands.drive.DriveToDistance;
import frc.robot.commands.teleopCommands.drive.TurnToAngle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.setSpeed;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final PhotonCamera camera = new PhotonCamera("FirebirdsCamera");
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(camera);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);
  private final XboxController m_operatorStick = new XboxController(IOConstants.kOperatorStick);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.driveArcade(
                m_driverStick.getRawAxis(Constants.IOConstants.kLY),
                m_driverStick.getRawAxis(Constants.IOConstants.kRX)),
            m_driveSubsystem));

    new JoystickButton(m_driverStick, IOConstants.kA).whileTrue(new Level0Setpoint(m_armSubsystem));
    new JoystickButton(m_driverStick, IOConstants.kB).whileTrue(new AmpSetpoint(m_armSubsystem));
    new JoystickButton(m_driverStick, IOConstants.kX).whileTrue(new SpeakerSetpoint(m_armSubsystem));

    new JoystickButton(m_driverStick, IOConstants.kRB).whileTrue(new IntakeNote(m_intake, m_shooterSubsystem));
    new JoystickButton(m_driverStick, IOConstants.kLB).whileTrue(new ShootNoteAmp(m_intake, m_shooterSubsystem));
    new JoystickButton(m_driverStick, IOConstants.kRB).onFalse(new StopIntake(m_intake));
    new JoystickButton(m_driverStick, IOConstants.kLB).onFalse(new StopIntake(m_intake));

    new JoystickButton(m_operatorStick, IOConstants.kY).whileTrue(new ClimbUp(m_climbSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kX).whileTrue(new ClimbDown(m_climbSubsystem));
    // new JoystickButton(m_driverStick, IOConstants.kY).onTrue(new TurnToAngle(m_driveSubsystem, 0, false));
    // new JoystickButton(m_driverStick, IOConstants.kA).onTrue(new PathPlannerAuto("AutoSpeaker1"));
    // new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new PathPlannerAuto("AutoSpeaker2"));
    // new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new PathPlannerAuto("AutoAmp1"));
    // new JoystickButton(m_operatorStick, IOConstants.kA).whileTrue(new AmpSetpoint(m_armSubsystem));
    // new JoystickButton(m_operatorStick, IOConstants.kB).whileTrue(new SpeakerSetpoint(m_armSubsystem));
    // new JoystickButton(m_operatorStick, IOConstants.kX).whileTrue(new Level0Setpoint(m_armSubsystem));
    //new JoystickButton(m_operatorStick, Constants.IOConstants.kA).onFalse(new StopIntake(m_intake));// kA
    //new JoystickButton(m_operatorStick, Constants.IOConstants.kA).whileTrue(new IntakeNote(m_intake));// kLB
  // new JoystickButton(m_driverStick, IOConstants.kA).whileTrue(new setSpeed(m_shooterSubsystem, 0));
  // new JoystickButton(m_driverStick, IOConstants.kB).whileTrue(new ShootAmp(m_shooterSubsystem));
  // new JoystickButton(m_driverStick, IOConstants.kX).whileTrue(new ShootSpeaker(m_shooterSubsystem));
  // new JoystickButton(m_driverStick, IOConstants.kY).whileTrue(new setSpeed(m_shooterSubsystem, -1000));


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
    // return m_chooser.getSelected();
  }
}
