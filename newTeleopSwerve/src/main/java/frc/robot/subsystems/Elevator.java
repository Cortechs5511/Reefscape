package frc.robot.subsystems;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorLeft = createElevatorController(ElevatorConstants.ELEVATOR_L_ID, false);
    private final SparkMax elevatorRight = createElevatorController(ElevatorConstants.ELEVATOR_R_ID, true);

    private XboxController operatorController = new XboxController(1);

    private final SparkAbsoluteEncoder TBEncoder = elevatorRight.getAbsoluteEncoder();

    private final RelativeEncoder relEncoder = elevatorLeft.getEncoder();

    private double cumulativeRotations = 0;
    private double previousPos = TBEncoder.getPosition();
    private double changeInPos = 0;


    public void setPower(double speed) {
        if (operatorController.getLeftY() < 0 && getAccumulatedRotations() > ElevatorConstants.MAX_POS) {
            elevatorLeft.set(0);
            elevatorRight.set(0);
        } else if (operatorController.getLeftY() > 0 && getAccumulatedRotations() < ElevatorConstants.MIN_POS) {
            elevatorLeft.set(0);
            elevatorRight.set(0);
        } else {
            if (operatorController.getLeftY() < 0 && getAccumulatedRotations() > ElevatorConstants.MAX_POS - 0.15) {
                elevatorLeft.set(speed* ElevatorConstants.SLOWED_POWER_UP);
                elevatorRight.set(speed*ElevatorConstants.SLOWED_POWER_UP);
            } else if (operatorController.getLeftY() > 0 && getAccumulatedRotations() < ElevatorConstants.MIN_POS + 0.15) {
                elevatorLeft.set(speed*ElevatorConstants.SLOWED_POWER_DOWN);
                elevatorRight.set(speed*ElevatorConstants.SLOWED_POWER_DOWN);
            } else {
                elevatorLeft.set(speed);
                elevatorRight.set(speed);
            }
        }   
    }

    public void setPosition(double position, boolean inInches) {
        if (inInches) {position /= 11.0;}

        if (getAccumulatedRotations() > position) {
            setPower(-1);
            if (Math.abs(getAccumulatedRotations() - position) < 0.15) {
                setPower(-ElevatorConstants.SLOWED_POWER_DOWN);
            }
        }
        else if (getAccumulatedRotations() < position) {
            setPower(1);
            if (Math.abs(getAccumulatedRotations() - position) < 0.15) {
                setPower(ElevatorConstants.SLOWED_POWER_UP);
            }
        }
    }

    private double getAccumulatedRotations() {
        double currentPos = TBEncoder.getPosition();

        if (Math.abs(currentPos - previousPos) < 0.5) {
            changeInPos = currentPos - previousPos;
        } else if (currentPos < previousPos) {
            changeInPos = currentPos + 1 - previousPos;
        } else {
            changeInPos = currentPos - 1 - previousPos;
        }

        previousPos = currentPos;
        cumulativeRotations += changeInPos;
        return cumulativeRotations;
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
// 11 inches per rotation

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/Raw Position", TBEncoder.getPosition());
        SmartDashboard.putNumber("Elevator/Accumulated Position", getAccumulatedRotations());
        SmartDashboard.putNumber("Elevator/Raw Velocity", TBEncoder.getVelocity());
        SmartDashboard.putNumber("Elevator/Relative Velocity", relEncoder.getVelocity());
        
        if (operatorController.getPOV() == 0) {
            cumulativeRotations = 0;
        }
        if (operatorController.getPOV() == 90) { 
            cumulativeRotations = 2.5; 
        }
    }
}
