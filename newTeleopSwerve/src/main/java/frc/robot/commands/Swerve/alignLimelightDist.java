package frc.robot.commands.Swerve;

import frc.robot.subsystems.Swerve.SwerveSubsystem;


import edu.wpi.first.wpilibj2.command.Command;

public class alignLimelightDist extends Command{
    private final SwerveSubsystem swerve;

    public alignLimelightDist(SwerveSubsystem subsystem) {
    swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        swerve.drive(swerve.limelightAlignDrive(), 0, 0, false, true, false);
        swerve.logStates();
    }

    @Override
    public boolean isFinished() { 
        return swerve.taIsAligned();
    }
}

