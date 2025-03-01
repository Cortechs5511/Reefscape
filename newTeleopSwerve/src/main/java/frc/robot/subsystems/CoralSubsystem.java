package frc.robot.subsystems;

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
    private final SparkMax flywheel = createCoralController(CoralConstants.CORAL_FLYWHEEL_ID, true);
    private final SparkMax wrist = createCoralController(CoralConstants.CORAL_WRIST_ID, false);

    private final SparkAbsoluteEncoder TBEncoder = wrist.getAbsoluteEncoder();

    private final RelativeEncoder wristEncoder = wrist.getEncoder();

    private void setCoralPower(double speed) {
        flywheel.set(speed);
    }    
    
    private void setWristPower(double speed) { 
        wrist.set(speed);
    }
    

    private double getRawPosition () {
        return wristEncoder.getPosition();
    }

    public void setSpeakerAngle(double power) {
        double rawPosition = getRawPosition(); 
        if ((power < 0 && rawPosition > CoralConstants.minimumPosition) || (power > 0 && rawPosition < CoralConstants.maximumPosition)) {
            setWristPower(power*0.5);
        } else {
            setWristPower(0);
        }

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
        SmartDashboard.putNumber("Coral/Raw Wrist Position", TBEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Raw Wrist Velocity", TBEncoder.getVelocity());
    }
}
