package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

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
    // private final PIDController[] drivePIDControllers;
    // private final PIDController[] turnPIDControllers;
    // private Gyro gyro;
    private SwerveDriveOdometry odometry;

    private Field2d field;

    private XboxController driveController;

    // public SwerveSubsystem() {
    //     modules = new SwerveModule[] {
    //     };
    // }

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
            config.idleMode(IdleMode.kCoast);
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

    // class Gyro {
    //     private AHRS navX;

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
