package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {
    private final PIDController wristPIDController = new PIDController(CoralConstants.WRIST_PID_VALUES[0], CoralConstants.WRIST_PID_VALUES[1], CoralConstants.WRIST_PID_VALUES[2]);

    private final SparkMax flywheel = createCoralController(CoralConstants.CORAL_FLYWHEEL_ID, true);
    private final SparkMax wrist = createCoralController(CoralConstants.CORAL_WRIST_ID, true);
    private final SparkMax angledFlywheel = createCoralController(CoralConstants.CORAL_ANGLED_FLYWHEEL_ID, false);

    private double pidOutput = 0 ;

    // dev
    private double outputPower;
    
    private XboxController operatorController = new XboxController(1);

    private final SparkAbsoluteEncoder TBEncoder = wrist.getAbsoluteEncoder();
    private final RelativeEncoder wristEncoder = wrist.getEncoder();

    public void setFlywheelPower(double intakePower, double outtakePower) {
        if (outtakePower != 0) {
            outputPower = outtakePower * CoralConstants.FLYWHEEL_OUTTAKE_MAX_POWER;  
            flywheel.set(outputPower);
            angledFlywheel.set(outputPower);
        } else {
            flywheel.set(intakePower);
            angledFlywheel.set(intakePower);
            // flywheel.set(intake ? CoralConstants.FLYWHEEL_INTAKE_MAX_POWER : 0);
        }
    }


    public void setWristPower(double speed) {
        if (speed > 0 && TBEncoder.getPosition() < CoralConstants.MAX_WRIST_POS) {
            wrist.set(0);
        } else if (speed < 0 && TBEncoder.getPosition() > CoralConstants.MIN_WRIST_POS) {
            wrist.set(0);
        } else if (TBEncoder.getPosition() > CoralConstants.PASSIVE_FALL_TOP && TBEncoder.getPosition() < CoralConstants.PASSIVE_FALL_BOT) {
            wrist.set((speed == 0) ? CoralConstants.PASSIVE_POWER : speed);
        }  else {
            wrist.set(speed);
        }
        SmartDashboard.putNumber("Coral/input speed", speed);
    }

    public void setWristPos(double pos) {
        if (TBEncoder.getPosition() < pos) {
            if (Math.abs(pos - TBEncoder.getPosition()) < 0.025) {
                setWristPower(-.05);
            } else {
                setWristPower(-.5);
            }
        } else {
            if (Math.abs(pos - TBEncoder.getPosition()) < 0.025) {
                setWristPower(.05);
            } else{ 
                setWristPower(0.5);
            }
        }
    }


    // PID for wrist
    public void setWristPosPID (double targetPos) {
        double currentPos = TBEncoder.getPosition();
        pidOutput = wristPIDController.calculate(currentPos, targetPos);
        setWristPower(-pidOutput);
        double error = Math.abs(currentPos - targetPos);
        
        // play around with error
        if (error < CoralConstants.ERROR_TOLERANCE) {
            setWristPower(0);
            wristPIDController.setSetpoint(targetPos);
            SmartDashboard.putBoolean("Coral/At Target", true);
        }
    }

    public boolean atWristPos (double targetPos) { 
        double error = Math.abs(TBEncoder.getPosition() - targetPos);
        return (error < CoralConstants.ERROR_TOLERANCE);
    }

    private SparkMax createCoralController(int port, boolean isInverted) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(CoralConstants.VOLTAGE_COMPENSATION);
        config.idleMode(CoralConstants.IDLE_MODE);
        config.openLoopRampRate(CoralConstants.RAMP_RATE);
        config.closedLoopRampRate(CoralConstants.RAMP_RATE);
        config.smartCurrentLimit(CoralConstants.CURRENT_LIMIT);
        config.inverted(isInverted);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return controller;
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral/Wrist PID output", pidOutput);
        SmartDashboard.putNumber("Coral/Raw Wrist Position", TBEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Raw Wrist Velocity", TBEncoder.getVelocity());
        SmartDashboard.putNumber("Coral/output power ", outputPower);
        
    }
}
