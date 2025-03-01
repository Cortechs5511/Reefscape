package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;

public class setElevatorPosition extends Command {
    private final Elevator elevator;
    private final OI oi = OI.getInstance();
    private final double targetPosition;
    private final boolean inInches;

    public setElevatorPosition(Elevator subsystem, double targetPosition, boolean inInches) {
        elevator = subsystem;
        addRequirements(subsystem);
        this.targetPosition = targetPosition;
        this.inInches = inInches;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.setPosition(targetPosition, inInches);
        SmartDashboard.putNumber("OI/Elevator Power", oi.getElevatorPower());
    }
}
