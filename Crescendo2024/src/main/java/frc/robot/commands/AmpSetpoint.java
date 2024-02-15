// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AmpSetpoint extends Command {

  private ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private double targetAngle = 140.00;

  //private final PIDController turnPID = new PIDController(0.102, 2.04, 0.001275);

  private double initialAngle;
  private double angle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AmpSetpoint(ArmSubsystem arm, double targetAngle) {
    m_armSubsystem = arm;
    angle = targetAngle;
    //turnPID.setTolerance(0.01, 1);
    initialAngle = m_armSubsystem.getAngle();
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double speed;
    SmartDashboard.putNumber("targetAngle", (initialAngle + targetAngle));
    //speed = turnPID.calculate(m_armSubsystem.getAngle(), initialAngle + targetAngle);
    m_armSubsystem.periodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
