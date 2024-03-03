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
  private final WPI_TalonSRX armPivotLeader = new WPI_TalonSRX(ArmConstants.ArmLeaderMotorCAN);
  private final WPI_VictorSPX armPivotFollower = new WPI_VictorSPX(ArmConstants.ArmFollowerMotorCAN);

  private final static FireBirdsUtils util = new FireBirdsUtils();

  // Target angles for arm
  private static double targetPosition = -200;

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
    armPivotLeader.configVoltageCompSaturation(12, 0);
    armPivotLeader.configPeakCurrentLimit(45);

    int ku = 45;
    double tu = 0.1;

    armPivotLeader.config_kP(0, 1.5);
    armPivotLeader.config_kI(0, 0);// 54
    armPivotLeader.config_kD(0, 0);// 3.374
    // Setting the velocity and acceleration of the motors
    armPivotLeader.configMotionCruiseVelocity(4000);
    armPivotLeader.configMotionAcceleration(500);

    armPivotLeader.configNeutralDeadband(0.04);


    if(Robot.isSimulation()){
      SmartDashboard.putData("Arm Sim", m_mech2d);
      m_armTower.setColor(new Color8Bit(Color.kBlue));
    }    

    // Configure the encoder
    armPivotLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    if (RobotBase.isSimulation()) {
      armPivotLeader.setSensorPhase(false);
      armPivotLeader.setInverted(true);
    } else {
      armPivotLeader.setSensorPhase(true);
      armPivotLeader.setInverted(true);
    }

    armPivotFollower.setInverted(InvertType.FollowMaster);

    // Configuring the limit switch
    armPivotLeader.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);

    armPivotLeader.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated,
        LimitSwitchNormal.NormallyOpen);
  }

  // Getting the position of the motor
  public double getAngle() {
    // Absolute position gets the location of the arm in ticks (4096 per revolution)
    return modAngleTicks(armPivotLeader.getSensorCollection().getQuadraturePosition() / ArmConstants.EncoderToOutputRatio);
  }

  public double getLeaderPower() {
    return armPivotLeader.get();
  }

  // Setter methods
  // Gives power
  public void setPower(double power) {
    armPivotLeader.set(power);
  }

  public void setNewTargetPosition(double newTargetPosition) {
    targetPosition = -newTargetPosition;
  }

  public boolean atZeroPos() {
    return armPivotLeader.isRevLimitSwitchClosed() == 0; // switch is open
  }

  public void stopMotor() {
    armPivotLeader.stopMotor();
  }

  public double modAngleTicks(double angleInTicks){
    return Math.IEEEremainder(angleInTicks, ArmConstants.EncoderCPR);
  }

  @Override
  public void periodic() {
    if (armPivotLeader.isRevLimitSwitchClosed() == 1) {// double check this value
      armPivotLeader.setSelectedSensorPosition(0);

    }

    double adjusted_feedForward = (ArmConstants.ArmShoulderFeedForward
        * Math.cos(util.CTRESensorUnitsToRads(targetPosition, ArmConstants.EncoderCPR)));

    SmartDashboard.putNumber("Target Position", targetPosition);
    SmartDashboard.putNumber("Adjusted feedforward", adjusted_feedForward);
    SmartDashboard.putNumber("Current Shoulder Position: ", getAngle());
    SmartDashboard.putNumber("Encoder value", armPivotLeader.getSensorCollection().getQuadraturePosition());

    if (RobotBase.isSimulation()) {
      armPivotLeader.set(
        ControlMode.MotionMagic, 
        targetPosition, 
        DemandType.ArbitraryFeedForward,
        adjusted_feedForward);
    } else {
      armPivotLeader.set(
        ControlMode.MotionMagic, 
        targetPosition * ArmConstants.EncoderToOutputRatio,
        DemandType.ArbitraryFeedForward,
        adjusted_feedForward);

    }
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(armPivotLeaderSim.getMotorOutputLeadVoltage());
    armSim.update(0.020);

    armPivotLeaderSim
        .setQuadratureRawPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), ArmConstants.EncoderCPR));

    m_arm.setAngle(Math.toDegrees(armSim.getAngleRads()));

    if (armSim.getAngleRads() == Units.degreesToRadians(ArmConstants.restDegreesFromHorizontal)) {

      armPivotLeader.getSensorCollection().setQuadraturePosition(0, 0);
    }

    armPivotLeaderSim.setAnalogPosition(util.radsToCTRESensorUnits(armSim.getAngleRads(), 4096));

  }
}