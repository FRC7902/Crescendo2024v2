// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;



public class ClimbSubsystem extends SubsystemBase {
    private final WPI_TalonSRX m_climbLeaderMotor = new WPI_TalonSRX(ClimbConstants.ClimbLeaderCAN);
    private final WPI_VictorSPX m_climbFollowerMotor = new WPI_VictorSPX(ClimbConstants.ClimbFollowerCAN);

    public ClimbSubsystem() {
        m_climbLeaderMotor.configFactoryDefault();
        m_climbFollowerMotor.configFactoryDefault();
        m_climbLeaderMotor.setInverted(false);
        m_climbFollowerMotor.setInverted(false);
        m_climbLeaderMotor.configContinuousCurrentLimit(ClimbConstants.constantCurrent);
        m_climbLeaderMotor.configPeakCurrentLimit(ClimbConstants.peakCurrent);
        m_climbFollowerMotor.follow(m_climbLeaderMotor);

        // Configure the encoder 
        // m_climbMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
        // 0);
    }
    
 
    // Stops motor
    public void stopMotor() {
        m_climbLeaderMotor.stopMotor();
    }

    // set motor power
    public void setPower(double power) {
        m_climbLeaderMotor.set(power);
    }


    @Override
    public void periodic() { 
        SmartDashboard.putNumber("Climber Current Limit", m_climbLeaderMotor.getSupplyCurrent());
    }

    @Override
    public void simulationPeriodic() {}
}
