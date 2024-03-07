// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    public static final int leftFrontCANID = 31;
    public static final int leftBackCANID = 32;
    public static final int rightFrontCANID = 33;
    public static final int rightBackCANID = 34;

    public static final double wheelDiameterMetres = 0.1524;
    public static final double gearRatio = 10.71;
    public static final double encoderTicksPerRevolution = 42;
    public static final double trackWidthInches = 24;

    public static final int GyroCAN = 2;

    public static final int PigeonCAN = 3;
  }

  public static final class ArmConstants {
    public static final int ArmLeaderMotorCAN = 21;// random
    public static final int ArmFollowerMotorCAN = 16;
    public static final int EncoderCPR = 4096;
    public static final double ArmShoulderFeedForward = 0.25;
    public static final double EncoderToOutputRatio = 2.05;
    public static final double restDegreesFromHorizontal = 90;
  }

  public final static class ShooterConstants {
    public static final int kMasterCAN = 30;
    public static final int kFollowerCAN = 35;

    public static final int[] kEncoderIDs = { 0, 0 };

    public final static double kHighSpeed = 0;
    public final static double kLowSpeed = 0;

    public final static double kRampTime = 0;

    public final static int kHighUnitsPerSec = 0;
    public final static int kTolerance = 0;
    // under this idk if will work, brake and coast speed?? pretty sure speed
    // difference for amp and speaker?
    // public final static int Brake = 0;
    // public final static int Coast = 0;
    public final static int kAmpSpeed = 3000;
    public final static int kSpeakerSpeed = 5000;

    public final static double shooterkD = 0;
    public final static double shooterkP = 0.01;
    public final static double shooterkI = 0;
  }

  public static class IntakeConstants {

    public static final int intakePWMid = 8;
    public static final double suckingSpeed = 1;
    public static final double spittingSpeed = -1;
    public static final double feedingSpeed = 1;
    public static final double holdPower = 0;
    public static final int beamBrake = 0;
    public static final double kSFeedForward = 0;
    public static final double kVFeedForward = 0;
    public static final double kAFeedForward = 0;
    public static final int intakeCurrentLimit = 15;
    public static final double intakeTargetSpeed = 1;

  }

    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
  
  public static final class IOConstants {
    public static final int kDriverStick = 0;
    public static final int kOperatorStick = 1;

    // Joystick Buttons
    public static final int kA = 1,
        kB = 2,
        kX = 3,
        kY = 4,
        kLB = 5,
        kRB = 6,
        kMENU = 7,
        kSTART = 8,
        kLA = 9,
        kRA = 10;

    // Joystick Axis
    public static final int kLX = 0,
        kLY = 1,
        kLT = 2,
        kRT = 3,
        kRX = 4,
        kRY = 5,
        kDX = 6,
        kDY = 7;

  }

  public static class ClimbConstants{
    public static final int ClimbCAN = 37;
    public static final double upSpeed = 1;
    public static final double downSpeed = -1;
    public static final int peakCurrent = 30; 
    public static final int constantCurrent = 15; 
  }
}

// all 0 need to be changed later, theyre place holders rn
