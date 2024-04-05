// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.autonomousCommands.OneNotePreload;
import frc.robot.commands.autonomousCommands.ThreeNoteAutoMiddleLeft;
import frc.robot.commands.autonomousCommands.ThreeNoteAutoMiddleRight;
import frc.robot.commands.autonomousCommands.TwoNoteAmpAutoClose;
import frc.robot.commands.autonomousCommands.TwoNoteAmpAutoFar;
import frc.robot.commands.autonomousCommands.TwoNoteAutoLeftSide;
import frc.robot.commands.autonomousCommands.FourNoteMiddle;
import frc.robot.commands.autonomousCommands.TwoNoteAutoMiddle;
import frc.robot.commands.autonomousCommands.TwoNoteAutoRightSide;
import frc.robot.commands.teleopCommands.arm.AmpSetpoint;
import frc.robot.commands.teleopCommands.arm.FeedingSetpoint;
import frc.robot.commands.teleopCommands.arm.Level0Setpoint;
import frc.robot.commands.teleopCommands.arm.MuteLimitSwitch;
import frc.robot.commands.teleopCommands.arm.SetAutoAimStatus;
import frc.robot.commands.teleopCommands.arm.SpeakerSetpoint;
import frc.robot.commands.teleopCommands.arm.decrementAngle;
import frc.robot.commands.teleopCommands.climb.ClimbDown;
import frc.robot.commands.teleopCommands.climb.ClimbUp;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.ShootNoteAmp;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.ShootNoteSpeaker;
import frc.robot.commands.teleopCommands.commandGroups.IntakeAndShooter.StopIntakeAndShooter;
import frc.robot.commands.teleopCommands.commandGroups.Scoring.ScoreNoteAmp;
import frc.robot.commands.teleopCommands.commandGroups.Scoring.ScoreNoteFeedAngle;
import frc.robot.commands.teleopCommands.commandGroups.Scoring.ScoreNoteSpeaker;
import frc.robot.commands.teleopCommands.intake.IntakeNote;
import frc.robot.commands.teleopCommands.intake.OuttakeNote;
import frc.robot.commands.teleopCommands.intake.ToggleOverrideBeamBrake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import org.photonvision.PhotonCamera;
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
  
  private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);
  private final XboxController m_operatorStick = new XboxController(IOConstants.kOperatorStick);

  private final PhotonCamera camera = new PhotonCamera("FirebirdsCamera");
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(camera);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_driveSubsystem);
  private final IntakeSubsystem m_intake = new IntakeSubsystem(m_operatorStick);
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();

  int autoDirection;

  private final TwoNoteAutoMiddle m_twoNoteMiddle;
  private final OneNotePreload m_oneNotePreload;
  private final ThreeNoteAutoMiddleRight m_ThreeNoteAutoMiddleRight;
  private final ThreeNoteAutoMiddleLeft m_ThreeNoteAutoMiddleLeft;
  private final TwoNoteAutoRightSide m_TwoNoteAutoRightSide;
  private final TwoNoteAutoLeftSide m_TwoNoteAutoLeftSide;
  private final FourNoteMiddle m_FourNoteMiddle;
  private final TwoNoteAmpAutoClose m_TwoNoteAmpAutoClose = new TwoNoteAmpAutoClose(m_driveSubsystem, m_armSubsystem, m_intake, m_shooterSubsystem);
  private final TwoNoteAmpAutoFar m_TwoNoteAmpAutoFar = new TwoNoteAmpAutoFar(m_driveSubsystem, m_armSubsystem, m_intake, m_shooterSubsystem);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> mirrorAuto = new SendableChooser<>();
  SendableChooser<Boolean> taxi = new SendableChooser<>();


  public RobotContainer() {
    //driverCamera.setDriverMode(true);
    configureBindings();

    mirrorAuto.setDefaultOption("No", 1);
    mirrorAuto.addOption("Yes", -1);
    SmartDashboard.putData("mirror auto?", mirrorAuto);

    autoDirection = mirrorAuto.getSelected();

    taxi.setDefaultOption("No", false);
    taxi.addOption("Yes", true);
    SmartDashboard.putData("taxi?", taxi);

    m_oneNotePreload = new OneNotePreload(m_driveSubsystem, m_armSubsystem, m_intake, m_shooterSubsystem);
    m_twoNoteMiddle = new TwoNoteAutoMiddle(m_driveSubsystem, m_intake, m_shooterSubsystem, m_armSubsystem, taxi.getSelected());
    m_TwoNoteAutoRightSide = new TwoNoteAutoRightSide(m_driveSubsystem, m_intake, m_shooterSubsystem, m_armSubsystem, autoDirection);
    m_TwoNoteAutoLeftSide = new TwoNoteAutoLeftSide(m_driveSubsystem, m_intake, m_shooterSubsystem, m_armSubsystem, autoDirection);
    m_ThreeNoteAutoMiddleRight = new ThreeNoteAutoMiddleRight(m_driveSubsystem, m_intake, m_armSubsystem, m_shooterSubsystem, autoDirection, taxi.getSelected());
    m_ThreeNoteAutoMiddleLeft = new ThreeNoteAutoMiddleLeft(m_driveSubsystem, m_intake, m_armSubsystem, m_shooterSubsystem, autoDirection, taxi.getSelected());
    m_FourNoteMiddle = new FourNoteMiddle(m_driveSubsystem, m_intake, m_armSubsystem, m_shooterSubsystem, autoDirection, taxi.getSelected());

    m_chooser.addOption("One Note Preload", m_oneNotePreload);
    m_chooser.addOption("Two Note Middle", m_twoNoteMiddle);
    m_chooser.addOption("Two Note Right", m_TwoNoteAutoRightSide);
    m_chooser.addOption("Two Note Left", m_TwoNoteAutoLeftSide);
    m_chooser.setDefaultOption("Three note right", m_ThreeNoteAutoMiddleRight);
    m_chooser.addOption("Three note left", m_ThreeNoteAutoMiddleLeft);
    m_chooser.addOption("Four Note Middle", m_FourNoteMiddle);
    m_chooser.addOption("Two note amp close", m_TwoNoteAmpAutoClose);
    m_chooser.addOption("Two note amp far", m_TwoNoteAmpAutoFar);
    SmartDashboard.putData("routine", m_chooser);

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
    new JoystickButton(m_operatorStick, IOConstants.kY).whileTrue(new FeedingSetpoint(m_armSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kLA).whileTrue(new decrementAngle(m_armSubsystem));
    // new JoystickButton(m_operatorStick, IOConstants.kY).whileTrue(new ScanField(m_driveSubsystem));
    // new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kLY) > 0.25).whileTrue(new ReelWinch(m_winchSubsystem, m_operatorStick.getRawAxis(IOConstants.kLY)));

    new JoystickButton(m_operatorStick, IOConstants.kRB).whileTrue(new IntakeNote(m_intake));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileTrue(new ConditionalCommand(
      new ShootNoteAmp(m_intake, m_shooterSubsystem),
      new ShootNoteSpeaker(m_intake, m_shooterSubsystem),
      m_armSubsystem::isArmAtAmp));;
    new JoystickButton(m_operatorStick, IOConstants.kRB).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kMENU).whileTrue(new ToggleOverrideBeamBrake(m_intake));
    new JoystickButton(m_operatorStick, IOConstants.kSTART).whileTrue(new SetAutoAimStatus(m_armSubsystem));

    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kRT) > 0.5).whileTrue(new ScoreNoteAmp(m_armSubsystem, m_shooterSubsystem, m_intake));
    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kLT) > 0.5).whileTrue(new ScoreNoteSpeaker(m_armSubsystem, m_shooterSubsystem, m_intake));
    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kRT) > 0.5).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));
    new Trigger(() -> m_operatorStick.getRawAxis(IOConstants.kLT) > 0.5).whileFalse(new StopIntakeAndShooter(m_intake, m_shooterSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kRA).whileTrue(new ScoreNoteFeedAngle(m_shooterSubsystem, m_armSubsystem, m_intake)); 

    new POVButton(m_operatorStick, 0).whileTrue(new ClimbUp(m_climbSubsystem));
    new POVButton(m_operatorStick, 180).whileTrue(new ClimbDown(m_climbSubsystem));

    // new POVButton(m_operatorStick, 0).whileTrue(new incrementAngle(m_armSubsystem));
    // new POVButton(m_operatorStick, 180).whileTrue(new decrementAngle(m_armSubsystem));
    
    new POVButton(m_operatorStick, 270).whileTrue(new OuttakeNote(m_intake));
    new POVButton(m_operatorStick, 90).whileTrue(new MuteLimitSwitch(m_armSubsystem));

    // new POVButton(m_operatorStick, 270).onTrue(new TurnToAngle(m_driveSubsystem, 0, false));
    // new POVButton(m_operatorStick, 90).onTrue(new TurnToAngle(m_driveSubsystem, -90, false));
    
    // new POVButton(m_operatorStick, 270).onTrue(new DriveToDistance(m_driveSubsystem, 3));
    // new POVButton(m_operatorStick, 90).onTrue(new DriveToDistance(m_driveSubsystem, -3));

  }

  public Command getAutonomousCommand() {
    m_oneNotePreload.setTaxi(taxi.getSelected());
    m_TwoNoteAutoLeftSide.setTaxi(taxi.getSelected());
    m_TwoNoteAutoRightSide.setTaxi(taxi.getSelected());
    return m_chooser.getSelected();
  }
}