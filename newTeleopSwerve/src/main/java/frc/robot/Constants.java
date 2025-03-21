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
    public static final int CORAL_FLYWHEEL_ID = 2;
    public static final int CORAL_WRIST_ID = 1;
    public static final int THROUGH_BORE_ID = 8;

    // MAX and MIN position of wrist
    public static final double MAX_WRIST_POS = 0.405;
    public static final double MIN_WRIST_POS = 0.89;
    public static final double PASSIVE_FALL_TOP = .5; // position where the wrist no longer falls down passively
    public static final double PASSIVE_FALL_BOT = .8; // position where we no longer need a passive boost to prevent wrist from falling
    public static final double PASSIVE_POWER = .01;
    public static final double WRIST_MAX_POWER = 0.3;
    public static final double[] WRIST_PID_VALUES = {5.75, 0.2, 0.0}; // change this when testing
    public static final double ERROR_TOLERANCE = 0;
    
    public static final double FLYWHEEL_INTAKE_MAX_POWER = 0.5;
    public static final double FLYWHEEL_OUTTAKE_MAX_POWER = -1.0;

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
    public static final double [] ELEVATOR_PID_VALUES = {1.2, 0.0, 0.0};
    public static final double ERROR_TOLERANCE = 0.05;



    public static final double MAX_POS = 3.54;
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
    
    
    public static final double[] DRIVE_PID_VALUES = {0.015, 0.0, 0.0};
    public static final double[] TURN_PID_VALUES = {.8, 0.0, 0.001};

    public static final double ANGLE_MAX_VELOCITY = 7.0;
    public static final double ANGLE_MAX_ACCELERATION = 30.0;
    

    // public static final double PID_RANGE = 0.9;
    // 18.7452 m 
    public static final double CHASSIS_LENGTH = Units.inchesToMeters(21.5);
    
    public static final double CHASSIS_WIDTH = Units.inchesToMeters(21.5);
    public static final double[] MODULE_TRANSLATIONS = {
      CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2, 
      CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
    };

    public static final double MAX_AUTON_SPEED = 4.5;
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(Math.sqrt(CHASSIS_LENGTH*CHASSIS_LENGTH+CHASSIS_WIDTH*CHASSIS_WIDTH));

    
    public static final double TRANSLATIONAL_SPEED_CONVERSION_FACTOR = 23.0;
    public static final double ROTATIONAL_SPEED_CONVERSION_FACTOR = 24.7;

    public static final double MAX_TRANSLATIONAL_SPEED = TRANSLATIONAL_SPEED_CONVERSION_FACTOR * 5.0;

    // Max rotational speed should be 3 / (Math.sqrt(2) * Units.inchesToMeters(23))
    // public static final double MAX_ROTATIONAL_SPEED = ROTATIONAL_SPEED_CONVERSION_FACTOR * 3.0 / (Math.sqrt(2.0) * CHASSIS_LENGTH / 2); 
    // public static final double MAX_ROTATIONAL_SPEED = ROTATIONAL_SPEED_CONVERSION_FACTOR * 30;
    public static final double MAX_ROTATIONAL_SPEED = 75;


    // Swerve Module Location Constants
    // each module is Math.sqrt(2) * Units.inchesToMeters(23) away from the center

  }

  public static class LimelightConstants {
    public static final double X_SETPOINT_REEF_ALIGNMENT = 2;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 2;
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.0;

    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.1;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.1;
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 2.5;

    public static final double X_SETPOINT_CUBE_ALIGNMENT = 0.0;
    public static final double Y_SETPOINT_CUBE_ALIGNMENT = 0.0;
    public static final double ROT_SETPOINT_CUBE_ALIGNMENT = 0.0;

    public static final double X_TOLERANCE_CUBE_ALIGNMENT = 0.1;
    public static final double Y_TOLERANCE_CUBE_ALIGNMENT = 0.1;
    public static final double ROT_TOLERANCE_CUBE_ALIGNMENT = 2.5;

    public static final double X_REEF_ALIGNMENT_P = 0.7;
    public static final double Y_REEF_ALIGNMENT_P = 0.7;
    public static final double ROT_REEF_ALIGNMENT_P = 0.7;

    public static final double DONT_SEE_TAG_WAIT_TIME = .5; // seconds
    public static final double POSE_VALIDATION_TIME = 0.4; // seconds
  }
}

