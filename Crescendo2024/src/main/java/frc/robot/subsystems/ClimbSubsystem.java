// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;



public class ClimbSubsystem extends SubsystemBase {
    // Declaring motor controllers
    private final WPI_TalonSRX m_climbMotor = new WPI_TalonSRX(ClimbConstants.ClimbCAN);
  


    public ClimbSubsystem() {
        m_climbMotor.configFactoryDefault();
       m_climbMotor.setInverted(false);
        m_climbMotor.configContinuousCurrentLimit(ClimbConstants.constantCurrent);
        m_climbMotor.configPeakCurrentLimit(ClimbConstants.peakCurrent);

        // Configure the encoder 
        // m_climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        // 0);
    }
    
 
    // Stops motor
    public void stopMotor() {
        m_climbMotor.stopMotor();
    }

    // set motor power
    public void setPower(double power) {
        m_climbMotor.set(power);
    }


    @Override
    public void periodic() { 
        SmartDashboard.putNumber("Climber Current Limit", m_climbMotor.getSupplyCurrent());
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }
}
