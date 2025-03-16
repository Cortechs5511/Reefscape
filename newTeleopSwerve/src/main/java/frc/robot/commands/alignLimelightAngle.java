package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.security.cert.X509CRL;

import edu.wpi.first.wpilibj2.command.Command;

public class alignLimelightAngle extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();

    public alignLimelightAngle(SwerveSubsystem subsystem) {
    swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        swerve.drive(0, 0, swerve.alignAngle(), false, true, false);
        swerve.logStates();
    }

    @Override 
    public boolean isFinished() {
        return swerve.angleIsAligned();
    }
}

