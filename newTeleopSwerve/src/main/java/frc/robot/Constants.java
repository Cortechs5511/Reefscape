// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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

  public static class CoralConstants {
    public static final int CORAL_L_ID = 60;
    public static final int CORAL_R_ID = 61;
    public static final int CORAL_W_ID = 61;
    public static final int THROUGH_BORE_ID = 8;

    public static final double VOLTAGE_COMPENSATION = 5;
    public static final int CURRENT_LIMIT = 40;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double RAMP_RATE = 0.05;

    public static final double maximumPosition = 0.32;
    public static final double minimumPosition = 0.0;

    public static final double POWER = 0.1 ;

  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_L_ID = 60;
    public static final int ELEVATOR_R_ID = 61;
    public static final int THROUGH_BORE_ID = 8;

    public static final double VOLTAGE_COMPENSATION = 5;
    public static final int CURRENT_LIMIT = 40;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double RAMP_RATE = 0.05;

    // Update later
    public static final double POSITION_CONVERSION_FACTOR = 50 * 22 / 12;

    // POWER STUFF
    public static final double MAX_POWER = 1.0;

    // approaching values
    public static final double SLOWED_POWER_UP = 0.05;
    public static final double SLOWED_POWER_DOWN = 0.075;



    public static final double MAX_POS = 2.5;
    // position of intake when down (intaking)
    public static final double MIN_POS = 0;

    public static final double RANGE = MAX_POS - MIN_POS;
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
    public static final double[] FL_DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] FL_TURN_PID_VALUES = {.7, 0.0, 0.0};

    public static final double[] FR_DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] FR_TURN_PID_VALUES = {.7, 0.0, 0.0};
    
    public static final double[] BL_DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] BL_TURN_PID_VALUES = {.7, 0.0, 0.0};
    
    public static final double[] BR_DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] BR_TURN_PID_VALUES = {.7, 0.0, 0.0};

    public static final double ANGLE_MAX_VELOCITY = 7.0;
    public static final double ANGLE_MAX_ACCELERATION = 30.0;
    

    // public static final double PID_RANGE = 0.9;

    public static final double MAX_TRANSLATIONAL_SPEED = 100 * 1.5;
    public static final double MAX_ROTATIONAL_SPEED = 75 * 1.5; 


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
