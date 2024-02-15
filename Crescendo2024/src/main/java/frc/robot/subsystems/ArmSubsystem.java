// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.FireBirdsUtils;

public class ArmSubsystem extends SubsystemBase {
  // Declaring motor controllers 
  private final WPI_TalonSRX armPivotLeader = new WPI_TalonSRX(ArmSubsystemConstants.ArmPivotLeaderCAN);
  private final WPI_VictorSPX armPivotFollower = new WPI_VictorSPX(ArmSubsystemConstants.ArmPivotFollowerCAN);
  
  private final static FireBirdsUtils util = new FireBirdsUtils();

  // Target angles for arm
  private static double targetPositionForAmp;
  private static double targetPositionForSpeaker;

  /** Object of a simulated arm **/
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getCIM(2), 
  139.78, 6.05, 1, 
  Units.degreesToRadians(-ArmSubsystemConstants.restDegreesFromHorizontal),
  6,true, 0);

  // Initializing the simulation of TalonSRX
  private final TalonSRXSimCollection armPivotLeaderSim = armPivotLeader.getSimCollection();

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(80, 80); // the overall arm
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30); // pivot point
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm = m_armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(armSim.getAngleRads()),
          6,
          new Color8Bit(Color.kAliceBlue)));

  // Creates a new ArmSubsystem
  public ArmSubsystem() {
    // idk what this does IDK WHAT THIS DOESSSSS
    armPivotLeader.configFactoryDefault();
    armPivotFollower.configFactoryDefault();

    armPivotFollower.follow(armPivotLeader);
    armPivotLeader.setInverted(false);
    armPivotFollower.setInverted(InvertType.FollowMaster);
    armPivotLeader.configVoltageCompSaturation(12,0);

    // Setting the value of P in PID control
    armPivotLeader.config_kP(0, 1.5);

    // Setting the velocity and acceleration of the motors
    armPivotLeader.configMotionCruiseVelocity(0);
    armPivotLeader.configMotionAcceleration(2);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

    // Configure the encoder
    armPivotLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        0);

    if (RobotBase.isSimulation()) {
      armPivotLeader.setInverted(false);
      armPivotLeader.setSensorPhase(false);
    } 
    else {
      armPivotLeader.setInverted(false);
      armPivotLeader.setSensorPhase(true);
    }

    // Configuring the limit switch
    armPivotLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
      LimitSwitchNormal.NormallyOpen);

    armPivotLeader.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, 
      LimitSwitchNormal.NormallyOpen);
  }

  // Getter methods

  // Getting the position of the motor
  public double getAngle() {
    // Absolute position gets the location of the arm in ticks (4096 per revolution)
    return armPivotLeader.getSelectedSensorPosition();
  }
  public double getLeaderPower() {
    return armPivotLeader.get();
  }

  // Setter methods
  // Gives power
  public void setPower(double power){
    armPivotLeader.set(power);
  }

  // Input two values that will be used for feedforward
  public void setPosition(double demand0, double demand1) {
    armPivotLeader.set(ControlMode.Position, demand0, DemandType.ArbitraryFeedForward, 0);
  }

  public void setNewAmpTargetPosition(double newAmpTargetPosition) {
    targetPositionForAmp = newAmpTargetPosition;
  }

  public void setNewSpeakerTargetPosition(double newSpeakerTargetPosition) {
    targetPositionForAmp = newSpeakerTargetPosition;
  }  

  public boolean atZeroPos() {
    return armPivotLeader.isRevLimitSwitchClosed() == 0; // switch is open
  }

  public void stopMotor() {
    armPivotLeader.stopMotor();
  }

  @Override
  public void periodic() {
    if (armPivotLeader.isRevLimitSwitchClosed() == 1) {
      armPivotLeader.setSelectedSensorPosition(0);

    }

    double currentPosition = getAngle(); //in raw sensor units

    double adjusted_feedForward = (ArmSubsystemConstants.ArmShoulderFeedForwardMin 
    * Math.cos(util.CTRESensorUnitsToRads(targetPositionForAmp, ArmSubsystemConstants.EncoderCPR)-
        ArmSubsystemConstants.angleAdjustmentRadians));

    SmartDashboard.putNumber("Current Arm Position: ", currentPosition);
    SmartDashboard.putNumber("Target Position", targetPositionForAmp);
    SmartDashboard.putNumber("Leader Voltage", armPivotLeader.getMotorOutputVoltage());
    SmartDashboard.putNumber("Adjusted feedforward", adjusted_feedForward);
    SmartDashboard.putNumber("Shoulder Current (A)", armPivotLeader.getSupplyCurrent());
    SmartDashboard.putNumber("Shoulder Error", armPivotLeader.getClosedLoopError(0));
    SmartDashboard.putNumber("Shoulder Position: ",  getAngle());
    //SmartDashboard.putBoolean("The arm is at amp shoot angle", ArmCommand.ampAngle());
    //SmartDashboard.putBoolean("The arm is at speaker shoot angle", ArmCommand.speakerAngle());

    if (RobotBase.isSimulation()) {
      armPivotLeader.set(ControlMode.MotionMagic, targetPositionForAmp, DemandType.ArbitraryFeedForward,
          adjusted_feedForward);
    } 
    else {
      armPivotLeader.set(ControlMode.MotionMagic, targetPositionForAmp*2, DemandType.ArbitraryFeedForward,
          adjusted_feedForward);

    }
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    armSim.setInput(armPivotLeaderSim.getMotorOutputLeadVoltage());
    // Next, we update it. The standard loop time is 20ms.
    armSim.update(0.020);

    armPivotLeaderSim
        .setQuadratureRawPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), ArmSubsystemConstants.EncoderCPR));

    m_arm.setAngle(Math.toDegrees(armSim.getAngleRads()));

    // Zero the limit switch in simulation
    if (armSim.getAngleRads() == Units.degreesToRadians(-ArmSubsystemConstants.restDegreesFromHorizontal)) {

      armPivotLeaderSim.getSensorCollection().setQuadraturePosition(0, 0);
    }

    armPivotLeaderSim.setAnalogPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), 4096));

  }
}