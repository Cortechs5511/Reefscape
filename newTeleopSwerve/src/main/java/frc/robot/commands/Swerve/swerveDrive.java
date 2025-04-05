package frc.robot.commands.Swerve;

import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class swerveDrive extends Command{
    private final SwerveSubsystem swerve;
    private final Elevator m_elevator;
    private final OI oi = OI.getInstance();
    private double xValue;
    private double yValue;
    private boolean fieldRelative; 
    private boolean turnWheel = true;
    private final Timer timer = new Timer();

    public swerveDrive(SwerveSubsystem subsystem, Elevator elevator) {
        swerve = subsystem;
        m_elevator = elevator;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        timer.start();
        fieldRelative = false;
    
        if (oi.driverPOV() == 180) {
            // if (timer.hasElapsed(.4)){
            //     xValue = -.1;
            //     if (oi.getDriverLeftBumper()) {
            //         xValue = -.05;
            //     }
            // } else {
            //     xValue = -0.001;
            // }
            
            if (oi.driverLeftTrigger() > 0) {
                if (timer.hasElapsed(.4)) {
                    xValue = -.05;
                } else {
                    xValue = -.001;
                }
            } else {
                xValue = -.1;
            }

        } else if (oi.driverPOV() == 0) {
            if (oi.driverLeftTrigger() > 0) {
                if (timer.hasElapsed(.4)) {
                    xValue = .05;
                } else {
                    xValue = .001;
                }
            } else {
                xValue = .1;
            }
        } else if (oi.driverPOV() == 90) {
            if (oi.driverLeftTrigger() > 0) {
                if (timer.hasElapsed(.4)) {
                    yValue = -.05;
                } else {
                    yValue = -.001;
                }
            } else {
                yValue = -.1;
            }
        } else if (oi.driverPOV() == 270) {
            if (oi.driverLeftTrigger() > 0) {
                if (timer.hasElapsed(.4)) {
                    yValue = .05;
                } else {
                    yValue = .001;
                }
            } else {
                yValue = .1;
            }
        } else {
            timer.restart();
            xValue = -oi.getDriverLeftY();
            yValue = -oi.getDriverLeftX();

            if (m_elevator.getAccumulatedRotations() > 3.0){
                xValue = xValue * 1/2;
                yValue = yValue * 1/2;
            } else if (m_elevator.getAccumulatedRotations() > 1.45) { 
                xValue = xValue * 4/5;
                yValue = yValue * 4/5;
            }
            fieldRelative = true;
        }

        SmartDashboard.putNumber("Swerve/X speed", xValue);
        SmartDashboard.putNumber("Swerve/Y speed", yValue);


        swerve.drive(xValue, yValue, -oi.getDriverRightX(), fieldRelative, false,  oi.driverX());

        swerve.logStates();
    }
}

