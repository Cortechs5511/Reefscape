package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.CoralSubsystem;

public class setWristPosition extends Command {
    private final CoralSubsystem coral;
    private final OI oi = OI.getInstance();
    private final double targetPosition;

    public setWristPosition(CoralSubsystem subsystem, double targetPosition) {
        coral = subsystem;
        addRequirements(subsystem);
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() { 
        // change to PID here
        coral.setWristPosPID(targetPosition);
        SmartDashboard.putNumber("OI/Elevator Power", oi.getElevatorPower());
    }
}
