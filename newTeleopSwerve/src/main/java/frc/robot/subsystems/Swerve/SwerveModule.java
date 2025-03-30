package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
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

    public void setTargetState(SwerveModuleState targetState, PIDController drivePID, ProfiledPIDController turnPID) {
        // get the current states
        currentState = new SwerveModuleState(getVelocity(), getAngle());

        // get angle + optimize angle 
        Rotation2d currentAngle = getAngle();
        targetState.optimize(currentAngle);

        // calculate the motor output + set the motor states
        double driveOutput = drivePID.calculate(getVelocity(), targetState.speedMetersPerSecond);
        double turnOutput = turnPID.calculate(getAngle().getRadians(), targetState.angle.getRadians());
        
        turnMotor.set(turnOutput);
        driveMotor.set(driveOutput);

        // set the states 
        double deltaTime = 0.02;
        double distanceChange = getVelocity() * deltaTime; 
        currentPosition = new SwerveModulePosition(
        currentPosition.distanceMeters + distanceChange,
        getAngle()
    );
    }

    // var delta = angle.minus(currentAngle);
    // if (Math.abs(delta.getDegrees()) > 90.0) {
    //   speedMetersPerSecond *= -1;
    //   angle = angle.rotateBy(Rotation2d.kPi);
    // }

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

    public SwerveModulePosition getPosition() {
        return currentPosition;
    }
}
