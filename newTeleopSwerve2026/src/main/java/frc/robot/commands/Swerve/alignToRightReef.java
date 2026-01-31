package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class alignToRightReef extends Command{
    private PIDController driveController; 
    private SwerveSubsystem swerve; 
    private Timer timer, stopTimer; 
    private Pose2d currentPose;
    private Pose2d targetPose; 


    public alignToRightReef (SwerveSubsystem subsystem) {
        driveController = new PIDController(LimelightConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
        swerve = subsystem; 
        addRequirements(subsystem);
    }

@Override
    public void initialize() {
        timer = new Timer(); 
        timer.start();
        
        stopTimer = new Timer();
        stopTimer.start();
        
        swerve.resetPose(new Pose2d());
        
        currentPose = swerve.getPose(); 
        targetPose = currentPose.plus(new Transform2d(0.0 , -.38, new Rotation2d().rotateBy(currentPose.getRotation())));

        SmartDashboard.putNumber("alignToRight/target pose angle", targetPose.getRotation().getDegrees());
        SmartDashboard.putNumber("alignToRight/current pose angle", currentPose.getRotation().getDegrees());


        driveController.setSetpoint(targetPose.getY());

        driveController.setTolerance(.02);
    }

    @Override
    public void execute() {

        if (!timer.hasElapsed(.4)) { 
            swerve.drive(0, .001 , 0, false, true, false);
        } else {
            double speed = driveController.calculate(swerve.getPose().getY()); 

            SmartDashboard.putNumber("alignToRight/Target Y position", driveController.getSetpoint());
            SmartDashboard.putNumber("alignToRight/current Y position", swerve.getPose().getY());
            SmartDashboard.putNumber("alignToRight/speed", speed);
            SmartDashboard.putNumber("alignToRight", stopTimer.get());
    
            swerve.drive(0, speed * 50 , 0, false, true, false);
    
            if (!driveController.atSetpoint()) {
            stopTimer.reset();
          } else {
          swerve.drive(0,0, 0, false, false, false);
        }
    }
    }


    @Override 
    public boolean isFinished () { 
        return (stopTimer.hasElapsed(.5));
    }
}
