package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class swerveDrive extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();
    private double xValue;
    private double yValue;
    private boolean fieldRelative; 
    private boolean turnWheel = true;

    public swerveDrive(SwerveSubsystem subsystem) {
        swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {

        
        
        fieldRelative = false;
        if (oi.driverPOV() == 180) {
            xValue = -.1;
            if (oi.getDriverLeftBumper()) {
                xValue = -.05;
            }
        } else if (oi.driverPOV() == 0) {
            xValue = .1;
            if (oi.getDriverLeftBumper()) {
                xValue = .05;
            }
        } else if (oi.driverPOV() == 90) {
            yValue = -.1;
            if (oi.getDriverLeftBumper()) {
                yValue = -.05;
            }
        } else if (oi.driverPOV() == 270) {
            yValue = .1;
            if (oi.getDriverLeftBumper()) {
                yValue = .05;
            }
        } else if (oi.driverStart()) {
            xValue = 0.001;
        } else if (oi.driverBack()) {
            yValue = 0.001;
        } else { 
            xValue = -oi.getDriverLeftY();
            yValue = -oi.getDriverLeftX();
            fieldRelative = true;
        }

        swerve.drive(xValue, yValue, -oi.getDriverRightX(), fieldRelative, false,  oi.driverX());

        // swerve.drive(0.5, 0, 0, false);
        swerve.logStates();
    }
}

