package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class buttonSwerveDrive extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();
    private double y;
    private double x;
    private double theta;

    public buttonSwerveDrive(SwerveSubsystem subsystem) {
        swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (oi.driverX()) {
            theta = 1;
        } else if (oi.driverB()) {
            theta = -1;
        } else {
            theta = 0;
        }

        if (oi.driverPOV() == -1) {
            y = 0;
            x = 0;
        } else {
            y = Math.cos(Math.toRadians(oi.driverPOV()));
            x = -Math.sin(Math.toRadians(oi.driverPOV()));
        }

        swerve.drive(y, x, theta, false, oi.driverY(), oi.driverA());
        swerve.logStates();
        SmartDashboard.putNumber("OI/Driver POV", oi.driverPOV());
    }

}
