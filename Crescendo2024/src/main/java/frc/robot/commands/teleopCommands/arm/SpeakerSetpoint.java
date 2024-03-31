// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCommands.arm;

import frc.robot.FireBirdsUtils;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SpeakerSetpoint extends Command {
  private final static FireBirdsUtils util = new FireBirdsUtils();
  private ArmSubsystem m_armSubsystem;
  private double targetAngle = ArmConstants.ArmSpeakerSetpoint;
  private double initialAngle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SpeakerSetpoint(ArmSubsystem arm) {
    m_armSubsystem = arm;
    initialAngle = m_armSubsystem.getAngle();
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setNewTargetPosition(util.degToCTRESensorUnits(targetAngle, ArmConstants.EncoderCPR));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.atTargetPosition();
  }
}
