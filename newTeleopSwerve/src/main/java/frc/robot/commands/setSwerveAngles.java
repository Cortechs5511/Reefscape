package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class setSwerveAngles extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();
    private double targetAngle;

    public setSwerveAngles(SwerveSubsystem subsystem, double targetAngle) {
        swerve = subsystem;
        addRequirements(subsystem);
        this.targetAngle = targetAngle;
    }

    @Override
    public void execute() {
        swerve.setAngles(targetAngle);
    }
}

