package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSubsystem;

public class setFlywheelPower extends Command {
    private final CoralSubsystem coral;
    private final OI oi = OI.getInstance();
    private final boolean intake;

    public setFlywheelPower(CoralSubsystem subsystem, boolean intake) {
        coral = subsystem;
        this.intake = intake;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // coral.setFlywheelPower(intake, oi.operatorRightTrigger());
        // SmartDashboard.putNumber("OI/Flywheel", oi.getOperatorRightY());
    }
}
