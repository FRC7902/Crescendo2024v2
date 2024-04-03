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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final WPI_VictorSPX armPivotFollower = new WPI_VictorSPX(ArmConstants.ArmFollowerMotorCAN);

  private final static FireBirdsUtils util = new FireBirdsUtils();
  private final DriveSubsystem m_driveSubsystem;

  // Target angles for arm
  private static double targetPosition = 0;
  private static int targetPositionCounter = 0;
  private static boolean isAutoAiming = false;
  private static boolean isManualControl = false;
  private static double adjusted_feedForward;

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
  public ArmSubsystem(DriveSubsystem drive) {
    m_driveSubsystem = drive;
    m_armLeaderMotor.configFactoryDefault();
    armPivotFollower.configFactoryDefault();

    armPivotFollower.follow(m_armLeaderMotor);
    m_armLeaderMotor.configVoltageCompSaturation(12, 0);
    m_armLeaderMotor.configPeakCurrentLimit(45);

    m_armLeaderMotor.config_kP(0, 5.5);
    m_armLeaderMotor.config_kI(0, 0); //0.00025
    m_armLeaderMotor.config_kD(0, 0.00045);// 3.374

    //tu = 0.5
    //ku = 7.5

    // m_armLeaderMotor.config_kP(0, 0.2 * 7.5);
    // m_armLeaderMotor.config_kI(0, 0.4 * 7.5 / 0.5);// 54
    // m_armLeaderMotor.config_kD(0, 0.066 * 7.5 * 0.5);// 3.374

    // Setting the velocity and acceleration of the motors
    m_armLeaderMotor.configMotionCruiseVelocity(200);
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

    armPivotFollower.setInverted(InvertType.FollowMaster);

    // Configuring the limit switch
    // m_armLeaderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
    //     LimitSwitchNormal.NormallyOpen);
  }

  // Getting the position of the motor
  public double getAngle() {
    // Absolute position gets the location of the arm in ticks (4096 per revolution)
    return modAngleTicks(m_armLeaderMotor.getSensorCollection().getQuadraturePosition() / ArmConstants.EncoderToOutputRatio);
  }

  public double getCurrentPosition(){
    return m_armLeaderMotor.getSensorCollection().getQuadraturePosition();
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

  public double getTargetPosition(){
    return -targetPosition;
  }

  public boolean atTargetPosition(){
    if(Math.abs(getAngle() - targetPosition) < 25){
      targetPositionCounter++;
    }else{
      targetPositionCounter = 0;
    }

    if(targetPositionCounter > 20){
      return true;
    }else{
      return false;
    }

  }

    public boolean notAtTargetPosition(){
    if(Math.abs(getAngle() - targetPosition) < 150){
      targetPositionCounter++;
    }else{
      targetPositionCounter = 0;
    }

    if(targetPositionCounter > 20){
      return false;
    }else{
      return true;
    }

  }

  // public boolean atZeroPos() {
  //   return m_armLeaderMotor.isRevLimitSwitchClosed() == 0; // switch is open
  // }

  public void stopMotor() {
    m_armLeaderMotor.stopMotor();
  }

  public double modAngleTicks(double angleInTicks){
    return Math.IEEEremainder(angleInTicks, ArmConstants.EncoderCPR);
  }
  
  public boolean isArmAtAmp(){
    return targetPosition - (-1 * (int) ArmConstants.ArmAmpSetpoint * ArmConstants.EncoderCPR/ 360) < 5;
  }

  public void setAutoAimingStatus(boolean status){
    isAutoAiming = status;
  }

  public double calculateAutoAim(){
    double displacement = m_driveSubsystem.getDistanceFromSpeaker();
    // double autoAngle = -54.831 * displacement * displacement + 381.77 * displacement - 59.147 + 33; //needs to be updated
    double autoAngle = ((-11.336 * displacement * displacement) - (51.952 * displacement) - 214.78 - 40);
    return autoAngle;
  }

  public void setManualControl(boolean manualControl){
    isManualControl = manualControl;
  }

  public double getFeedforward(){
    return adjusted_feedForward;
  }

  @Override
  public void periodic() {
    if(DriverStation.isDisabled()){
      setNewTargetPosition(0);
      // m_armLeaderMotor.setSelectedSensorPosition(m_armLeaderMotor.getSensorCollection().getAnalogInRaw());
    }

    // if(m_armLeaderMotor.isRevLimitSwitchClosed() == 1) {
    //   m_armLeaderMotor.getSensorCollection().setQuadraturePosition(0, 1);
    // }

    adjusted_feedForward = (ArmConstants.ArmShoulderFeedForward
        * Math.cos(-util.CTRESensorUnitsToRads(getAngle(), ArmConstants.EncoderCPR) - 0.07));

    //SmartDashboard.putNumber("Adjusted feedforward", adjusted_feedForward);
    SmartDashboard.putNumber("feedforward", adjusted_feedForward);
    SmartDashboard.putNumber("Current Shoulder Position: ", getAngle());
    SmartDashboard.putBoolean("at target position", atTargetPosition());
    SmartDashboard.putNumber("TARGET POSITION", targetPosition);
    // SmartDashboard.putBoolean("Arm Limit Switch", m_armLeaderMotor.isRevLimitSwitchClosed() == 1);
    // SmartDashboard.putBoolean("fws Limit Switch", m_armLeaderMotor.isFwdLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("is auto aiming", isAutoAiming);
    SmartDashboard.putBoolean("is manual control", isManualControl);
    SmartDashboard.putNumber("angle in radians", -util.CTRESensorUnitsToRads(getAngle(), ArmConstants.EncoderCPR) - 0.21);
    SmartDashboard.putNumber("autoAngle", 1.3 * calculateAutoAim());
    // SmartDashboard.putNumber("s", calculateAutoAim() - getAngle());
    // SmartDashboard.putNumber("d", calculateAutoAim() / getAngle());

    if (RobotBase.isSimulation()) {
      m_armLeaderMotor.set(
        ControlMode.MotionMagic, 
        targetPosition, 
        DemandType.ArbitraryFeedForward,
        adjusted_feedForward);
    } else {
      if(targetPosition == 0 && atTargetPosition() && !isAutoAiming){
        m_armLeaderMotor.set(0);
      }else{
        if(isAutoAiming){
          m_armLeaderMotor.set(
            ControlMode.MotionMagic, 
            1.3 * calculateAutoAim() *  ArmConstants.EncoderToOutputRatio,
            DemandType.ArbitraryFeedForward,
            adjusted_feedForward);
        }else if(!isManualControl){
          m_armLeaderMotor.set(
            ControlMode.MotionMagic, 
            targetPosition * ArmConstants.EncoderToOutputRatio,
            DemandType.ArbitraryFeedForward,
            adjusted_feedForward);
        }
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
}