package frc.robot.subsystems;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    // When testing, figure out which motor must be inverted
    private final SparkMax elevatorLeft = createElevatorController(ElevatorConstants.ELEVATOR_L_ID, false);
    private final SparkMax elevatorRight = createElevatorController(ElevatorConstants.ELEVATOR_R_ID, true);

    private final PIDController ElevatorPID = new PIDController(ElevatorConstants.PID_VALUES[0], ElevatorConstants.PID_VALUES[1],
        ElevatorConstants.PID_VALUES[2]);

    //private final SparkAbsoluteEncoder TBEncoder = createTBEncoder(elevatorRight);
    private final SparkAbsoluteEncoder TBEncoder = elevatorRight.getAbsoluteEncoder();


    public void setElevatorPosition(double targetPosition) {
        double elevatorOutput = ElevatorPID.calculate(getConvertedValue(), targetPosition);
        setPower(elevatorOutput);
    }

    public void setPower(double speed) {
        elevatorLeft.set(speed);
        elevatorRight.set(speed);
    }
    
    private double getConvertedValue() { 
        return TBEncoder.getPosition() * ElevatorConstants.POSITION_CONVERSION_FACTOR;
    }

    private SparkMax createElevatorController(int port, boolean isInverted) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(ElevatorConstants.VOLTAGE_COMPENSATION);
        config.idleMode(ElevatorConstants.IDLE_MODE);
        config.openLoopRampRate(ElevatorConstants.RAMP_RATE);
        config.closedLoopRampRate(ElevatorConstants.RAMP_RATE);
        config.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        config.inverted(isInverted);
        config.encoder.positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return controller;
    }

    private SparkAbsoluteEncoder createTBEncoder(SparkMax encoderMotor) {
        SparkAbsoluteEncoder encoder = encoderMotor.getAbsoluteEncoder();
        return encoder;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Raw Position", TBEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Converted Position", getConvertedValue());
        SmartDashboard.putNumber("Elevator/Raw Velocity", TBEncoder.getVelocity());
        System.out.println(TBEncoder.getPosition());
    }
}
