package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class setCoralPower extends Command {
    private final CoralSubsystem coral;
    private final OI oi = OI.getInstance();
    private double power;

    public setCoralPower(CoralSubsystem subsystem) {
        coral = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coral.setWristPower(oi.getOperatorRightY() * CoralConstants.WRIST_MAX_POWER);
        if (oi.getOperatorPOV() == -1) {
            power = 0; 
        } else if (oi.getOperatorPOV() == 0) {
            power = 0.2;
        } else if (oi.getOperatorPOV() == 90) {
            power = .4;
        }  else if (oi.getOperatorPOV() == 180) {
            power = .6;
        } else if (oi.getOperatorPOV() == 270) {
            power = .8;
        } else {
            power = 0;
        }
        coral.setFlywheelPower(power, oi.operatorRightTrigger());
        SmartDashboard.putNumber("OI/Wrist Power", oi.getOperatorRightY());
    }
}
