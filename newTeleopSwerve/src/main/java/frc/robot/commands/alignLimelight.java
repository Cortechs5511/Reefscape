package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class alignLimelight extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();

    public alignLimelight(SwerveSubsystem subsystem) {
        swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        swerve.limelightAlignStrafe();
        // swerve.drive(0.5, 0, 0, false);
        swerve.logStates();
    }
}

