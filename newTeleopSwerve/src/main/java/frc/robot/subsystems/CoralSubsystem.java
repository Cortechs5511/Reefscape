package frc.robot.subsystems;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
// processor, algea l1, algea l2, coral l1, coral l2, coral l3, corall4, intake station 
// A B X Y, LEFT JOYSTICK, RIGHT JOYSTICK BUTTON, MENU button
// a
public class CoralSubsystem extends SubsystemBase {
    private final SparkMax coralLeft = createCoralController(CoralConstants.CORAL_L_ID, true);
    private final SparkMax coralRight = createCoralController(CoralConstants.CORAL_R_ID, false);
    private final SparkMax wrist = createCoralController(CoralConstants.CORAL_W_ID, false);

    private final RelativeEncoder wristEncoder = wrist.getEncoder();

    private void setCoralPower(double speed) {
        coralLeft.set(speed);
        coralRight.set(speed);
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

}