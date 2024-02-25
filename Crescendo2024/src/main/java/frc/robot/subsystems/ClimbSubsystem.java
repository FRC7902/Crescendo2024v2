// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
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
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.teleopCommands.ClimbDistance;


public class ClimbSubsystem extends SubsystemBase {
    // Declaring motor controllers
    private final WPI_TalonSRX m_climbMotor = new WPI_TalonSRX(ClimbConstants.ClimbLeaderCAN);
    // Initializing the simulation of TalonSRX
    private final TalonSRXSimCollection m_climbSim = m_climbMotor.getSimCollection();
    // // Initialize the motor that powers the climbing system
    // private final CANSparkMax m_climbMotor = new CANSparkMax(DriveConstants.leftFrontCANID,
    //   CANSparkMax.MotorType.kBrushless);
    // Initialize the encoder attached to the motor
    Encoder encoder = new Encoder(0, 1);
    private double target;
    private final PIDController climbPID = new PIDController(0.5, 0, 0);
    /** Object of a simulated evalator system**/
    // idk how to make two line objects appear where one just moves up and one stays in place

    // Create a Mechanism2d display of an elevator climb system with a fixed tower and moving beam.
    // IDK HOW TO PROGRAM IT THO

    public ClimbSubsystem() {
        m_climbMotor.configFactoryDefault();
       m_climbMotor.setInverted(false);
       m_climbMotor.configVoltageCompSaturation(12,0);
        m_climbMotor.configPeakCurrentLimit(45);

        // Setting the velocity and acceleration of the motors
      m_climbMotor.configMotionCruiseVelocity(8000);
      m_climbMotor.configMotionAcceleration(2000);

        // Put simulated object to SmartDashboard


        // Configure the encoder
        m_climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        0);


        if (RobotBase.isSimulation()) {
            m_climbMotor.setInverted(false);
           m_climbMotor.setSensorPhase(false);
        }
        else {
          m_climbMotor.setInverted(false);
           m_climbMotor.setSensorPhase(true);
        }
    }
    
    // Returns the distance travelled calculated by the encoder
    public double encoderDistance(){
        return encoder.getDistance();
    }
 
    // Stops motor
    public void stopMotor() {
        m_climbMotor.stopMotor();
    }

    // set motor power
    public void setPower() {
        m_climbMotor.set(1);
    }

    public void setTargetPosition(double targetInInches){
        target = targetInInches;
    }

    public boolean atZeroPos() {
        return m_climbMotor.isRevLimitSwitchClosed() == 0; // switch is open
      }

    @Override
    public void periodic() { 
        if (m_climbMotor.isRevLimitSwitchClosed() == 1) {// double check this value
            m_climbMotor.setSelectedSensorPosition(0);
      
          }
        
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Encoder value", encoder.getDistance());
        SmartDashboard.putBoolean("Limit Switch at ZERO", atZeroPos());
        m_climbMotor.set(climbPID.calculate(encoder.getDistance(), target));
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
