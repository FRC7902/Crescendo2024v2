// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpSetpoint;
import frc.robot.commands.SpeakerSetpoint;
import frc.robot.subsystems.ArmSubsystem;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Basic;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
        private static final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();


        // Replace with CommandPS4Controller or CommandJoystick if needed

        // THE FIRST CONTOLLER PLUGGED IN CONTROLS THE DRIVETRAIN, THE SECOND CONTROLLER
        // PLUGGED IN CONTROLS THE ARM/INTAKE
        private final XboxController m_driverStick = new XboxController(IOConstants.kDriverStick);
        private final XboxController m_operatorStick = new XboxController(IOConstants.kOperatorStick);

        // The container for the robot. Contains subsystems, OI devices, and commands.
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

    // Button that when pressed down it brings arm to shooting angle
    new JoystickButton(m_operatorStick, IOConstants.kLB).whileTrue(new AmpSetpoint(m_ArmSubsystem));
    new JoystickButton(m_operatorStick, IOConstants.kLA).whileTrue(new SpeakerSetpoint(m_ArmSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  // }
}
