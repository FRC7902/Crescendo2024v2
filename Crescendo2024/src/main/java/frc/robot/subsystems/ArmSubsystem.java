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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.FireBirdsUtils;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  // Declaring motor controllers
  private final WPI_TalonSRX m_armLeaderMotor = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);
  private final WPI_VictorSPX m_armFollowerMotor = new WPI_VictorSPX(ArmConstants.ArmFollowerMotorCAN);

  private final static FireBirdsUtils util = new FireBirdsUtils();

  // Target angles for arm
  private static double targetPosition = 0;

  /** Object of a simulated arm **/
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getCIM(2),
      139.78,
      6.05,
      1,
      -Math.PI * 20,
      Math.PI * 20,
      true,
      0);

  // Initializing the simulation of TalonSRX
  private final TalonSRXSimCollection m_armLeaderMotorSim = m_armLeaderMotor.getSimCollection();

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
    m_armLeaderMotor.configFactoryDefault();
    m_armFollowerMotor.configFactoryDefault();

    m_armFollowerMotor.follow(m_armLeaderMotor);
    m_armLeaderMotor.configVoltageCompSaturation(12, 0);
    m_armLeaderMotor.configPeakCurrentLimit(45);

    int ku = 45;
    double tu = 0.1;

    m_armLeaderMotor.config_kP(0, 1.5);
    m_armLeaderMotor.config_kI(0, 0);// 54
    m_armLeaderMotor.config_kD(0, 0);// 3.374
    // Setting the velocity and acceleration of the motors
    m_armLeaderMotor.configMotionCruiseVelocity(2500);
    m_armLeaderMotor.configMotionAcceleration(500);

    m_armLeaderMotor.configNeutralDeadband(0.04);


    if(Robot.isSimulation()){
      SmartDashboard.putData("Arm Sim", m_mech2d);
      m_armTower.setColor(new Color8Bit(Color.kBlue));
    }    

    // Configure the encoder
    m_armLeaderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    if (RobotBase.isSimulation()) {
      m_armLeaderMotor.setSensorPhase(false);
      m_armLeaderMotor.setInverted(true);
    } else {
      m_armLeaderMotor.setSensorPhase(true);
      m_armLeaderMotor.setInverted(true);
    }

    m_armFollowerMotor.setInverted(InvertType.FollowMaster);

    // Configuring the limit switch
    m_armLeaderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);

    m_armLeaderMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated,
        LimitSwitchNormal.NormallyOpen);
  }

  @Override
  public void periodic() {
    if (m_armLeaderMotor.isRevLimitSwitchClosed() == 1) {// double check this value
      m_armLeaderMotor.setSelectedSensorPosition(0);

    }

    double adjusted_feedForward = (ArmConstants.ArmShoulderFeedForward
        * Math.cos(util.CTRESensorUnitsToRads(targetPosition, ArmConstants.EncoderCPR)));

    SmartDashboard.putNumber("Target Position", targetPosition);
    SmartDashboard.putNumber("Adjusted feedforward", adjusted_feedForward);
    SmartDashboard.putNumber("Current Shoulder Position: ", getAngle());
    SmartDashboard.putNumber("Encoder value", m_armLeaderMotor.getSensorCollection().getQuadraturePosition());

    if (RobotBase.isSimulation()) {
      m_armLeaderMotor.set(
        ControlMode.MotionMagic, 
        targetPosition, 
        DemandType.ArbitraryFeedForward,
        adjusted_feedForward);
    } else {
      if(targetPosition == 0){
        m_armLeaderMotor.set(0);
      }else{
        m_armLeaderMotor.set(
          ControlMode.MotionMagic, 
          targetPosition * ArmConstants.EncoderToOutputRatio,
          DemandType.ArbitraryFeedForward,
          adjusted_feedForward);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(m_armLeaderMotorSim.getMotorOutputLeadVoltage());
    armSim.update(0.020);

    m_armLeaderMotorSim
        .setQuadratureRawPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), ArmConstants.EncoderCPR));

    m_arm.setAngle(Math.toDegrees(armSim.getAngleRads()));

    if (armSim.getAngleRads() == Units.degreesToRadians(ArmConstants.restDegreesFromHorizontal)) {

      m_armLeaderMotor.getSensorCollection().setQuadraturePosition(0, 0);
    }

    m_armLeaderMotorSim.setAnalogPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), 4096));

  }

  // Getting the position of the motor
  public double getAngle() {
    // Absolute position gets the location of the arm in ticks (4096 per revolution)
    return modAngleTicks(m_armLeaderMotor.getSensorCollection().getQuadraturePosition() / ArmConstants.EncoderToOutputRatio);
  }

  public double getLeaderPower() {
    return m_armLeaderMotor.get();
  }

  // Setter methods
  // Gives power
  public void setPower(double power) {
    m_armLeaderMotor.set(power);
  }

  public void setNewTargetPosition(double newTargetPosition) {
    targetPosition = -newTargetPosition;
  }

  public boolean isArmAtAmp(){
    return targetPosition == ArmConstants.ArmAmpSetpoint;
  }

  public boolean atZeroPos() {
    return m_armLeaderMotor.isRevLimitSwitchClosed() == 0; // switch is open
  }

  public void stopMotor() {
    m_armLeaderMotor.stopMotor();
  }

  public double modAngleTicks(double angleInTicks){
    return Math.IEEEremainder(angleInTicks, ArmConstants.EncoderCPR);
  }
}