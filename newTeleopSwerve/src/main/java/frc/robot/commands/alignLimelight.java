package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.security.cert.X509CRL;

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
        // swerve.drive(oi.getDriverLeftY() * 1.5, swerve.limelightAlignStrafe(), 0, false, false, false);
        swerve.drive(0, swerve.limelightAlignStrafe(), 0, false, true, false, false);
        swerve.logStates();
    }
}

