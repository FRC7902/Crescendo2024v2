// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.teleopCommands.command_groups.ShootNoteAmp;
import frc.robot.commands.teleopCommands.command_groups.ShootNoteSpeaker;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.teleopCommands.arm.AmpSetpoint;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.climb.ClimbDown;
import frc.robot.commands.teleopCommands.climb.ClimbUp;
import frc.robot.commands.teleopCommands.drive.ScanField;
import frc.robot.commands.teleopCommands.intake.IntakeNote;
import frc.robot.commands.teleopCommands.intake.StopIntake;
import frc.robot.commands.teleopCommands.shooter.StopShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.driveArcade(
                m_driverStick.getRawAxis(Constants.IOConstants.kLY),
                m_driverStick.getRawAxis(Constants.IOConstants.kRX)),
            m_driveSubsystem));

    new JoystickButton(m_operatorStick, IOConstants.kA).whileTrue(new Level0Setpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kB).whileTrue(new AmpSetpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kX).whileTrue(new SpeakerSetpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kY).whileTrue(new ScanField(m_driveSubsystem));

    new JoystickButton(m_operatorStick, IOConstants.kRB).whileTrue(new IntakeNote(m_intake));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileTrue(new ConditionalCommand(
      new ShootNoteAmp(m_intake, m_shooterSubsystem),
      new ShootNoteSpeaker(m_intake, m_shooterSubsystem),
      m_armSubsystem::isArmAtAmp));
    new JoystickButton(m_operatorStick, IOConstants.kRB).whileFalse(new StopIntake(m_intake));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileFalse(new StopShooter(m_shooterSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kRB).whileFalse(new StopShooter(m_shooterSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileFalse(new StopIntake(m_intake));

    new JoystickButton(m_operatorStick, IOConstants.kMENU).whileTrue(new ClimbUp(m_climbSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kSTART).whileTrue(new ClimbDown(m_climbSubsystem));
    // new JoystickButton(m_driverStick, IOConstants.kY).onTrue(new TurnToAngle(m_driveSubsystem, 0, false));
    // new JoystickButton(m_driverStick, IOConstants.kA).onTrue(new PathPlannerAuto("AutoSpeaker1"));
    // new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new PathPlannerAuto("AutoSpeaker2"));
    // new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new PathPlannerAuto("AutoAmp1"));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
    // return m_chooser.getSelected();
  }
}
