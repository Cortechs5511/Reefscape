package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private final PIDController[] drivePIDControllers;
    private final PIDController[] turnPIDControllers;
    // Properties for Field oriented driving
    // private Gyro gyro;
    // private SwerveDriveOdometry odometry;
    // private Field2d field;

    private XboxController driveController;

    public SwerveSubsystem() {
        //gyro =
        // Initialize the swerve modules
        modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.IDS[0], SwerveConstants.IDS[1], SwerveConstants.IDS[2]),
            new SwerveModule(SwerveConstants.IDS[3], SwerveConstants.IDS[4], SwerveConstants.IDS[5]),
            new SwerveModule(SwerveConstants.IDS[6], SwerveConstants.IDS[7], SwerveConstants.IDS[8]),
            new SwerveModule(SwerveConstants.IDS[9], SwerveConstants.IDS[10], SwerveConstants.IDS[11]),
        };
        // Initalize kinematic objects
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[0], SwerveConstants.MODULE_TRANSLATIONS[1]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[2], SwerveConstants.MODULE_TRANSLATIONS[3]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[4], SwerveConstants.MODULE_TRANSLATIONS[5]),
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[6], SwerveConstants.MODULE_TRANSLATIONS[7])
        );
        // initialize PIDidontrollers 
        drivePIDControllers = new PIDController[] {
            new PIDController(SwerveConstants.FL_DRIVE_PID_VALUES[0], SwerveConstants.FL_DRIVE_PID_VALUES[1], SwerveConstants.FL_DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.FR_DRIVE_PID_VALUES[0], SwerveConstants.FR_DRIVE_PID_VALUES[1], SwerveConstants.FR_DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.BL_DRIVE_PID_VALUES[0], SwerveConstants.BL_DRIVE_PID_VALUES[1], SwerveConstants.BL_DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.BR_DRIVE_PID_VALUES[0], SwerveConstants.BR_DRIVE_PID_VALUES[1], SwerveConstants.BR_DRIVE_PID_VALUES[2])
        };

        turnPIDControllers = new PIDController[] {
            new PIDController(SwerveConstants.FL_TURN_PID_VALUES[0], SwerveConstants.FL_TURN_PID_VALUES[1], SwerveConstants.FL_TURN_PID_VALUES[2]),
            new PIDController(SwerveConstants.FR_TURN_PID_VALUES[0], SwerveConstants.FR_TURN_PID_VALUES[1], SwerveConstants.FR_TURN_PID_VALUES[2]),
            new PIDController(SwerveConstants.BL_TURN_PID_VALUES[0], SwerveConstants.BL_TURN_PID_VALUES[1], SwerveConstants.BL_TURN_PID_VALUES[2]),
            new PIDController(SwerveConstants.BR_TURN_PID_VALUES[0], SwerveConstants.BR_TURN_PID_VALUES[1], SwerveConstants.BR_TURN_PID_VALUES[2])  
        };

        for (PIDController p : turnPIDControllers) {
            p.enableContinuousInput(0, 2 * Math.PI);
        };

        // For Field driving later 
        // odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
        // field = new Field2d();

        driveController = new XboxController(0);
    }

    @Override 
    public void periodic () {
    // transfering the controller inputs into SwerveModuleState
        ChassisSpeeds controllerSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * -driveController.getLeftY(), 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * -driveController.getLeftX(), 
            SwerveConstants.MAX_ROTATIONAL_SPEED * -driveController.getRightX()
        );
        SwerveModuleState[] controllerStates = kinematics.toSwerveModuleStates(controllerSpeeds);
        
        // Logging the swerve Module state data onto the smart module
        double[] controllerStatesAsDoubles = {
            controllerStates[0].angle.getRadians(),
            controllerStates[0].speedMetersPerSecond,
            controllerStates[1].angle.getRadians(),
            controllerStates[1].speedMetersPerSecond,
            controllerStates[2].angle.getRadians(),
            controllerStates[2].speedMetersPerSecond,
            controllerStates[3].angle.getRadians(),
            controllerStates[3].speedMetersPerSecond,
        };
        SmartDashboard.putNumberArray("Controller State", controllerStatesAsDoubles);

        logStates();
    }


    public void drive(double y, double x, double theta, boolean fieldRelative, boolean resetPID, boolean resetGyro) {
        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * y, 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * x,
            SwerveConstants.MAX_ROTATIONAL_SPEED * theta
        );

        if (resetPID) {
            for (int i = 0; i < 4; i++) {
                turnPIDControllers[i].reset();
            }
        }
        // reset gyro button
        // if (resetGyro) {
        //     gyro.resetGyro();
        // }

        // implementing field logic
        // if (fieldRelative) {
        //     driveFieldRelative(newDesiredSpeeds);
        // }
        // else {
        //     driveRobotRelative(newDesiredSpeeds);
        // }

        driveRobotRelative(newDesiredSpeeds);
    }

    // public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    //     driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    // }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        // add constant for max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_TRANSLATIONAL_SPEED);

        modules[0].setTargetState(targetStates[0], drivePIDControllers[0], turnPIDControllers[0]);
        modules[1].setTargetState(targetStates[1], drivePIDControllers[1], turnPIDControllers[1]);
        modules[2].setTargetState(targetStates[2], drivePIDControllers[2], turnPIDControllers[2]);
        modules[3].setTargetState(targetStates[3], drivePIDControllers[3], turnPIDControllers[3]);
    //    for (int i = 0; i<4; i++) {
    //     modules[i].setTargetState(targetStates[i], drivePIDControllers[i], turnPIDControllers[i]);
    //    }
    
    }

    public void logStates() {
        double[] loggingState = {
            modules[0].getAngle().getRadians(),
            modules[0].getVelocity(),
            modules[1].getAngle().getRadians(),
            modules[1].getVelocity(),
            modules[2].getAngle().getRadians(),
            modules[2].getVelocity(),
            modules[3].getAngle().getRadians(),
            modules[3].getVelocity(),
        };
        
        SmartDashboard.putNumberArray("Module State", loggingState);
        // SmartDashboard.putNumber("Gyro Radians", gyro.getRotation2d().getRadians());

        SmartDashboard.putNumber("Swerve/FL Velocity", modules[0].getVelocity());
        SmartDashboard.putNumber("Swerve/FR Velocity", modules[1].getVelocity());
        SmartDashboard.putNumber("Swerve/BL Velocity", modules[2].getVelocity());
        SmartDashboard.putNumber("Swerve/BR Velocity", modules[3].getVelocity());
    }

    class SwerveModule {
        private SwerveModulePosition currentPosition;
        private SwerveModuleState currentState;
        private SparkMax driveMotor;
        private SparkMax turnMotor;
        private CoreCANcoder absoluteEncoder;

        private RelativeEncoder driveEncoder;


        public SwerveModule(int driveMotorPort, int turningMotorPort, int absoluteEncoderPort) {
            driveMotor = createMotorController(driveMotorPort, false);
            turnMotor = createMotorController(turningMotorPort, false);
            absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);

            driveEncoder = createEncoder(driveMotor);

            currentState = new SwerveModuleState(getVelocity(), getAngle());
            currentPosition = new SwerveModulePosition();
        }

        private SparkMax createMotorController(int port, boolean isInverted) {
            SparkMax controller = new SparkMax(port, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
    
            config.voltageCompensation(SwerveConstants.VOLTAGE_COMPENSATION);
            config.idleMode(IdleMode.kBrake);
            config.openLoopRampRate(SwerveConstants.RAMP_RATE);
            config.closedLoopRampRate(SwerveConstants.RAMP_RATE); 
    
            config.smartCurrentLimit(SwerveConstants.CURRENT_LIMIT);
    
            config.inverted(isInverted);
    
            config.encoder.velocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
            // for some reason causes robot to shake:
            //     controller.burnFlash(); 
            controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            return controller;
        }

        private RelativeEncoder createEncoder(SparkMax controller) {
            RelativeEncoder encoder = controller.getEncoder();
            // convert from native unit of rpm to m/s
            //encoder.setVelocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
    
            return encoder;
        }

        public void setTargetState(SwerveModuleState targetState, PIDController drivePID, PIDController turnPID) {
            Rotation2d currentAngle = getAngle();
            targetState.optimize(currentAngle);
            // currentState = targetState;
            double driveOutput = drivePID.calculate(getVelocity(), targetState.speedMetersPerSecond);
            double turnOutput = turnPID.calculate(getAngle().getRadians(), targetState.angle.getRadians());
            
            turnMotor.set(turnOutput);
            driveMotor.set(driveOutput);
        }

        public double getVelocity() {
            return driveEncoder.getVelocity();
        }

        public double getAbsoluteEncoderPos() {
            return absoluteEncoder.getAbsolutePosition().getValueAsDouble(); 
        }

        public Rotation2d getAngle() {
            return Rotation2d.fromRotations(getAbsoluteEncoderPos());
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(getVelocity(), getAngle());
        }
    }

    //class Gyro {
    //    private AHRS navX;

    //     public Gyro() {
    //         navX = new AHRS(NavXComType.kMXP_SPI);  
    //     }

    //     public Rotation2d getRotation2d() {
    //         // Get yaw in degrees from the navX
    //         double yawDegrees = navX.getYaw(); 
            
    //         // Convert degrees to radians (Rotation2d uses radians)
    //         double yawRadians = Math.toRadians(yawDegrees);
            
    //         // Create and return a Rotation2d object
    //         return new Rotation2d(yawRadians);
    //     }

    //     public void resetGyro() {
    //         navX.reset();
    //     }

    // }
}