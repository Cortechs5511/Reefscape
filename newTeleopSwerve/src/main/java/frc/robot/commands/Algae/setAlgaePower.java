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
    private double flywheelPower;

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
            algae.setWristPower(wristPower * AlgaeConstants.WRIST_MAX_POWER);
        } else if (oi.operatorPOV() == 180) { 
            wristPower = -1; 
            algae.setWristPower(wristPower * AlgaeConstants.WRIST_MAX_POWER);
            flywheelPower = 1.0;
        } else {
            wristPower = 0.0;
            flywheelPower = 0;
            algae.setWristPosPID(0);
        }

        algae.setFlywheelPower(flywheelPower);
        SmartDashboard.putNumber("OI/Wrist Power", oi.getOperatorRightY());
    }
}
