// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

  public final static class ArmSubsystemConstants{
    // CAN ID
    public final static int ArmPivotLeaderCAN = 0;
    public final static int ArmPivotFollowerCAN = 1;

    // Simulation Constants
    public final static double restDegreesFromHorizontal = 75;

     // OPERATIONAL
     public final static double ArmShoulderRotatePower = 0.5; // testing, not final
     public final static double ArmShoulderStop = 0.0; // testing, not final
     public final static double ArmShoulderFeedForwardMin = 0.26; // final and tested
     public final static double ArmShoulderFeedForwardMax = 0.5; // final and tested
     public final static double ArmShoulderFeedForwardDifference = ArmShoulderFeedForwardMax - ArmShoulderFeedForwardMin;
     public static final double ShoulderBufferTimeInSeconds = 0.75;

     // REAL WORLD CONSTANTS
    public final static double angleAdjustmentDegrees = 71.57;
    public final static double angleAdjustmentRadians = Units.degreesToRadians(angleAdjustmentDegrees);
    public final static double EncoderToOutputRatio = 0.5;
    public final static int EncoderCPR = 4096;
    public final static double ticksPerDegree = EncoderCPR / 360;
  }
}
