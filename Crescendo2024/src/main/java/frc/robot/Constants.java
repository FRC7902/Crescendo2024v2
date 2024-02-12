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

  public static final class DriveConstants{
    public static final int leftFrontCANID = 31;
    public static final int leftBackCANID = 32;
    public static final int rightFrontCANID = 33;
    public static final int rightBackCANID = 34;

    public static final int GyroCAN = 2;

    public static final int PigeonCAN = 3;
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IOConstants {
    public static final int kDriverStick = 0;
    public static final int kLB = 5, kRB = 6;
    public static final int kA = 1, kB = 2, kX = 3, kY = 4;
    public static final int kLX = 0, kLY = 1, kRX = 2, kRY = 3;
  }
}
