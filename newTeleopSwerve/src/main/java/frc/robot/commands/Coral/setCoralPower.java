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
    private double outtakePower;

    public setCoralPower(CoralSubsystem subsystem) {
        coral = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        coral.setWristPower(oi.getOperatorRightY()  * CoralConstants.WRIST_MAX_POWER);

        if (oi.getOperatorRightBumper()) {
            power = .2;
        } else {
            power = 0.0;
        }

        if (oi.operatorRightTrigger() > 0) { 
            outtakePower = 1.0;
        } else if (oi.operatorLeftTrigger() > 0 ) { 
            outtakePower = .4;
        } else {
            outtakePower = 0.0;
        }

        coral.setFlywheelPower(power, outtakePower);
        SmartDashboard.putNumber("OI/Wrist Power", oi.getOperatorRightY());
    }
}
