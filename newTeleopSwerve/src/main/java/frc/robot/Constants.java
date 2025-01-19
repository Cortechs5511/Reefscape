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

  public static class SwerveConstants {
    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 40;
    public static final double RAMP_RATE = 0.05;
    public static final double WHEEL_DIAMETER_IN = 4;
    public static final double WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN*Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double INCHES_PER_METER = 39.3701;
    // convert native units of rpm to meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER / 60;
  }
}
