// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

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
    private final WPI_TalonSRX armPivotLeader = new WPI_TalonSRX(ClimbConstants.ClimbLeaderCAN);
    // Initializing the simulation of TalonSRX
    private final TalonSRXSimCollection armPivotLeaderSim = armPivotLeader.getSimCollection();
    // Initialize the motor that powers the climbing system
    private final CANSparkMax m_climbMotor = new CANSparkMax(DriveConstants.leftFrontCANID,
      CANSparkMax.MotorType.kBrushless);
    // Initialize the encoder attached to the motor
    Encoder encoder = new Encoder(0, 1);
   
    /** Object of a simulated evalator system**/
    // idk how to make two line objects appear where one just moves up and one stays in place

    // Create a Mechanism2d display of an elevator climb system with a fixed tower and moving beam.
    // IDK HOW TO PROGRAM IT THO

    public ClimbSubsystem() {
        armPivotLeader.configFactoryDefault();
        armPivotLeader.setInverted(false);
        armPivotLeader.configVoltageCompSaturation(12,0);
        armPivotLeader.configPeakCurrentLimit(45);

        // Setting the velocity and acceleration of the motors
        armPivotLeader.configMotionCruiseVelocity(8000);
        armPivotLeader.configMotionAcceleration(2000);

        // Put simulated object to SmartDashboard


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
    }
    
    // Returns the distance travelled calculated by the encoder
    public double encoderDistance(){
        return encoder.getDistance();
    }
 
    // Stops motor
    public void stopMotor() {
        armPivotLeader.stopMotor();
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        SmartDashboard.putNumber("Encoder value", encoder.getDistance());
        if(encoder.getDistance() == 0){
            m_climbMotor.set(ClimbConstants.speed);
            // power the motor to get to ClimbConstants.distance
        }        
        else{
            stopMotor();
        }
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
