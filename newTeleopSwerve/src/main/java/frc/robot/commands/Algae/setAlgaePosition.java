package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeSubsystem;

public class setAlgaePosition extends Command {
    private final AlgaeSubsystem algae;
    private final OI oi = OI.getInstance();
    private double position;

    public setAlgaePosition(AlgaeSubsystem subsystem, double targetPosition) {
        algae = subsystem;
        position = targetPosition;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        algae.setWristPosPID(position);
        algae.setFlywheelPower(1);
    }

    @Override
    public void execute() {

        algae.setFlywheelPower(oi.operatorLeftTrigger());
        SmartDashboard.putNumber("OI/Wrist Power", oi.getOperatorRightY());
    }
}
