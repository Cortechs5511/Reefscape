package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.security.cert.X509CRL;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class alignLimelight extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();
    private final Timer timer = new Timer();
    private double speed = 0 ; 


    public alignLimelight(SwerveSubsystem subsystem) {
    swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        // if (oi.driverA()) {
        //     timer.start();
        //     if (timer.hasElapsed(.5)) {
        //         speed = swerve.limelightAlignStrafe();
        //         swerve.drive(0, speed, 0, false, true, false);
        //     } else { 
        //         speed = 0.001;
        //         swerve.drive(0, speed, 0, false, true, false);

        //     }
        // } 
        speed = swerve.limelightAlignStrafe();
        swerve.drive(0, speed, 0, false, true, false);
        // swerve.drive(oi.getDriverLeftY() * 1.5, swerve.limelightAlignStrafe(), 0, false, false, false);        swerve.logStates();
    }

    @Override 
    public boolean isFinished() {
        return swerve.txIsAligned();
    }
}

