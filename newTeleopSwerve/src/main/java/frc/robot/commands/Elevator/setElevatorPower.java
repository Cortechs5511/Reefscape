package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class setElevatorPower extends Command {
    private final Elevator elevator;
    private final OI oi = OI.getInstance();

    public setElevatorPower(Elevator subsystem) {
        elevator = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.setPower(oi.getElevatorPower() * ElevatorConstants.MAX_POWER);
        SmartDashboard.putNumber("OI/Elevator Power", oi.getElevatorPower());
    }
}
