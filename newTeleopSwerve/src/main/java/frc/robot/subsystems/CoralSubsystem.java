package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;

public class CoralSubsystem extends SubsystemBase {
    private final SparkMax flywheel = createCoralController(CoralConstants.CORAL_FLYWHEEL_ID, true);
    private final SparkMax wrist = createCoralController(CoralConstants.CORAL_WRIST_ID, false);

    // dev
    private double outputPower;
    
    private XboxController operatorController = new XboxController(1);

    private final SparkAbsoluteEncoder TBEncoder = wrist.getAbsoluteEncoder();

    private final RelativeEncoder wristEncoder = wrist.getEncoder();

    public void setFlywheelPower(double intakePower, double outtakePower) {
        if (outtakePower != 0) {
            outputPower = outtakePower * CoralConstants.FLYWHEEL_OUTTAKE_MAX_POWER;  
            flywheel.set(outputPower);
        } else {
            flywheel.set(intakePower);
            // flywheel.set(intake ? CoralConstants.FLYWHEEL_INTAKE_MAX_POWER : 0);
        }
    }


    public void setWristPower(double speed) {
        // if (operatorController.getLeftY() < 0 && getRawPosition() > CoralConstants.MAX_WRIST_POS) {
        //     wrist.set(0);
        // } else if (operatorController.getLeftY() > 0 && getRawPosition() < CoralConstants.MIN_WRIST_POS) {
        //     wrist.set(0);
        // } else {
        wrist.set(speed);
        // }   
    }

    private double getRawPosition () {
        return wristEncoder.getPosition();
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
        SmartDashboard.putNumber("Coral/output power ", outputPower);
        
    }
}
