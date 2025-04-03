package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class setAlgaePower extends Command {
    private final AlgaeSubsystem algae;
    private final OI oi = OI.getInstance();
    private double power;
    private double wristPower;

    public setAlgaePower(AlgaeSubsystem subsystem) {
        algae = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (oi.operatorPOV() == 0) { 
            wristPower = 1; 
        } else if (oi.operatorPOV() == 180) { 
            wristPower = -1; 
        } else {
            wristPower = 0.0;
        }

        algae.setWristPower(wristPower * AlgaeConstants.WRIST_MAX_POWER);

        algae.setFlywheelPower(oi.operatorLeftTrigger());
        SmartDashboard.putNumber("OI/Wrist Power", oi.getOperatorRightY());
    }
}
