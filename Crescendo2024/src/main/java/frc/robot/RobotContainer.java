// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.autonomousCommands.DriveOut;
import frc.robot.commands.autonomousCommands.LeaveHome;
import frc.robot.commands.autonomousCommands.LeaveNoteOnGroundLeaveHome;
import frc.robot.commands.autonomousCommands.TwoNoteAutoSimple;
import frc.robot.commands.teleopCommands.arm.AmpSetpoint;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.climb.ClimbDown;
import frc.robot.commands.teleopCommands.climb.ClimbUp;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.ShootNoteAmp;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.ShootNoteSpeaker;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.commandGroups.Scoring.ScoreNoteAmp;
import frc.robot.commands.teleopCommands.commandGroups.Scoring.ScoreNoteSpeaker;
import frc.robot.commands.teleopCommands.drive.DriveRaw;
import frc.robot.commands.teleopCommands.drive.DriveToDistance;
import frc.robot.commands.teleopCommands.drive.ScanField;
import frc.robot.commands.teleopCommands.drive.TurnToAngle;
import frc.robot.commands.teleopCommands.intake.IntakeNote;
import frc.robot.commands.teleopCommands.intake.StopIntake;
import frc.robot.commands.teleopCommands.shooter.StopShooter;
import frc.robot.commands.teleopCommands.drive.DriveRaw;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_driveSubsystem);
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final TwoNoteAutoSimple m_simpleTwoNote = new TwoNoteAutoSimple(m_driveSubsystem, m_intake, m_shooterSubsystem, m_armSubsystem);
  private final DriveOut m_simpleOneNote = new DriveOut(m_driveSubsystem, m_armSubsystem, m_intake, m_shooterSubsystem);
  private final LeaveHome m_leaveHome = new LeaveHome(m_driveSubsystem);
  private final LeaveNoteOnGroundLeaveHome m_LeaveNoteOnGroundLeaveHome = new LeaveNoteOnGroundLeaveHome(m_armSubsystem, m_intake, m_shooterSubsystem, m_driveSubsystem);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);
  private final XboxController m_operatorStick = new XboxController(IOConstants.kOperatorStick);

  public RobotContainer() {
    //driverCamera.setDriverMode(true);
    configureBindings();

    m_chooser.setDefaultOption("Simple Two Note", m_simpleTwoNote);
    m_chooser.addOption("Simple One Note", m_simpleOneNote);
    m_chooser.addOption("Drive Only", m_leaveHome);
    m_chooser.addOption("AMp", m_LeaveNoteOnGroundLeaveHome);
    SmartDashboard.putData(m_chooser);
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

    new JoystickButton(m_operatorStick, IOConstants.kA).whileTrue(new Level0Setpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kB).whileTrue(new AmpSetpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kX).whileTrue(new SpeakerSetpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kY).whileTrue(new ScanField(m_driveSubsystem));

    new JoystickButton(m_operatorStick, IOConstants.kRB).whileTrue(new IntakeNote(m_intake));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileTrue(new ConditionalCommand(
      new ShootNoteAmp(m_intake, m_shooterSubsystem),
      new ShootNoteSpeaker(m_intake, m_shooterSubsystem),
      m_armSubsystem::isArmAtAmp));;
    new JoystickButton(m_operatorStick, IOConstants.kRB).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));

    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kRT) > 0.5).whileTrue(new ScoreNoteAmp(m_armSubsystem, m_shooterSubsystem, m_intake));
    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kLT) > 0.5).whileTrue(new ScoreNoteSpeaker(m_armSubsystem, m_shooterSubsystem, m_intake));
    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kRT) > 0.5).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));
    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kLT) > 0.5).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));

    new POVButton(m_operatorStick, 0).whileTrue(new ClimbUp(m_climbSubsystem));
    new POVButton(m_operatorStick, 180).whileTrue(new ClimbDown(m_climbSubsystem));

    // new POVButton(m_operatorStick, 90).whileTrue(new DriveToDistance(m_driveSubsystem, 0.5));//POSITIVE VALUE GOES BACKWARDS
    // new POVButton(m_operatorStick, 270).whileTrue(new DriveToDistance(m_driveSubsystem, -0.5));

    // new JoystickButton(m_driverStick, IOConstants.kY).onTrue(new TurnToAngle(m_driveSubsystem, 0, false));
    // new JoystickButton(m_driverStick, IOConstants.kA).onTrue(new PathPlannerAuto("AutoSpeaker1"));
    // new JoystickButton(m_driverStick, IOConstants.kB).onTrue(new PathPlannerAuto("AutoSpeaker2"));
    // new JoystickButton(m_driverStick, IOConstants.kX).onTrue(new PathPlannerAuto("AutoAmp1"));

  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
