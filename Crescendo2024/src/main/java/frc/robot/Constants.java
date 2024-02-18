// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
    
    public final static class ShooterConstants {
      public static final int kMasterCAN = 1;
      public static final int kFollowerCAN = 0;

      public static final int[] kEncoderIDs = {0, 0};
      
      public final static double kHighSpeed = 0;
      public final static double kLowSpeed = 0;

      public final static double kRampTime = 0;

      public final static int kHighUnitsPerSec = 0;
      public final static int kTolerance = 0;
      // under this idk if will work, break and coast speed?? pretty sure speed difference for amp and speaker? 
      //public final static int Brake = 0; 
      //public final static int Coast = 0; 
      public final static int kAmpSpeed = 0; 
      public final static int kSpeakerSpeed = 0; 
  }

  public final static class IOConstants{
    public static final int kDriverStick = 0;
    public static final int kOperatorStick = 1;

    // Joystick Buttons
    public static final int kA = 1,
        kB = 2,
        kX = 3,
        kY = 4,
        kLB = 5,
        kRB = 6,
        kBACK = 7,
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
}

// all 0 need to be changed later, theyre place holders rn 
