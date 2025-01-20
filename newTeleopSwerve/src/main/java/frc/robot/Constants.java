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

  public static class OIConstants {

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DEADBAND = 0.1;
  }

  public static class SwerveConstants {

    // Motor Controller and Encoder Configuration
    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 40;
    public static final double RAMP_RATE = 0.05;
    public static final double WHEEL_DIAMETER_IN = 4;
    public static final double WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN*Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double INCHES_PER_METER = 39.3701;
    // convert native units of rpm to meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER / 60;

    // IDs
    public static final int[] IDS = {12, 11, 1, 22, 21, 2, 32, 31, 3, 42, 41, 4};

    // PID Values
    public static final double[] FL_DRIVE_PID_VALUES = {0.02, 0.0, 0.0};
    public static final double[] FL_TURN_PID_VALUES = {0.05, 0.0, 0.0};

    public static final double[] FR_DRIVE_PID_VALUES = {0.02, 0.0, 0.0};
    public static final double[] FR_TURN_PID_VALUES = {0.05, 0.0, 0.0};
    
    public static final double[] BL_DRIVE_PID_VALUES = {0.02, 0.0, 0.0};
    public static final double[] BL_TURN_PID_VALUES = {0.05, 0.0, 0.0};
    
    public static final double[] BR_DRIVE_PID_VALUES = {0.02, 0.0, 0.0};
    public static final double[] BR_TURN_PID_VALUES = {0.05, 0.0, 0.0};

    // public static final double PID_RANGE = 0.9;

    public static final double MAX_TRANSLATIONAL_SPEED = 100;
    public static final double MAX_ROTATIONAL_SPEED = 75;


    // Swerve Module Location Constants
    public static final double CHASSIS_LENGTH = Units.inchesToMeters(23);
    public static final double CHASSIS_WIDTH = Units.inchesToMeters(23);
    public static final double[] MODULE_TRANSLATIONS = {
      CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2, 
      CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
    };

    public static final double MAX_AUTON_SPEED = 4.5;
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(Math.sqrt(CHASSIS_LENGTH*CHASSIS_LENGTH+CHASSIS_WIDTH*CHASSIS_WIDTH));

  }
}
