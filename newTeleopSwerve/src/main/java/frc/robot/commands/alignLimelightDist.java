package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.security.cert.X509CRL;

import edu.wpi.first.wpilibj2.command.Command;

public class alignLimelightDist extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();

    public alignLimelightDist(SwerveSubsystem subsystem) {
    swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        swerve.drive(swerve.limelightAlignDrive(), 0, 0, false, true, false);
        swerve.logStates();
    }
}

