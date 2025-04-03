package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {
    private final PIDController wristPIDController = new PIDController(AlgaeConstants.ALGAE_WRIST_P, AlgaeConstants.ALGAE_WRIST_I, AlgaeConstants.ALGAE_WRIST_D);

    private final SparkMax flywheel = createCoralController(AlgaeConstants.ALGAE_FLYWHEEL_ID, true);
    private final SparkMax wrist = createCoralController(AlgaeConstants.ALGAE_WRIST_ID, false);

    private XboxController operatorController = new XboxController(1);

    private final RelativeEncoder wristEncoder = wrist.getEncoder();

    public void setFlywheelPower(double power) {
        flywheel.set(power);
    }  


    public void setWristPower(double speed) {
        // if (speed > 0 && TBEncoder.getPosition() < AlgaeConstants.MAX_WRIST_POS) {
        //     wrist.set(0);
        // } else if (speed < 0 && TBEncoder.getPosition() > AlgaeConstants.MIN_WRIST_POS) {
        //     wrist.set(0);
        // } else if (TBEncoder.getPosition() > CoralConstants.PASSIVE_FALL_TOP && TBEncoder.getPosition() < AlgaeConstants.PASSIVE_FALL_BOT) {
        //     wrist.set((speed == 0) ? CoralConstants.PASSIVE_POWER : speed);
        // }  else {
        //     wrist.set(speed);
        // }
        
        if (speed == 0) {
            wrist.set(0);
        } else {
            wrist.set(speed);
        }
        SmartDashboard.putNumber("Coral/input speed", speed);
    }


    // // PID for wrist
    // public void setWristPosPID (double targetPos) {
    //     double currentPos = TBEncoder.getPosition();
    //     pidOutput = wristPIDController.calculate(currentPos, targetPos);
    //     setWristPower(-pidOutput);
    //     double error = Math.abs(currentPos - targetPos);
        
    //     // play around with error
    //     if (error < CoralConstants.ERROR_TOLERANCE) {
    //         setWristPower(0);
    //         wristPIDController.setSetpoint(targetPos);
    //         SmartDashboard.putBoolean("Coral/At Target", true);
    //     }
    // }

    // public boolean atWristPos (double targetPos) { 
    //     double error = Math.abs(TBEncoder.getPosition() - targetPos);
    //     return (error < CoralConstants.ERROR_TOLERANCE);
    // }

    private SparkMax createCoralController(int port, boolean isInverted) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(AlgaeConstants.VOLTAGE_COMPENSATION);
        config.idleMode(AlgaeConstants.IDLE_MODE);
        config.openLoopRampRate(AlgaeConstants.RAMP_RATE);
        config.closedLoopRampRate(AlgaeConstants.RAMP_RATE);
        config.smartCurrentLimit(AlgaeConstants.CURRENT_LIMIT);
        config.inverted(isInverted);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        return controller;
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae/Wrist Encoder", wristEncoder.getPosition());
    }
}
