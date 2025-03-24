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
    private Timer timer; 
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
    }

    @Override
    public void execute() {
        if (!timer.hasElapsed(.3)) {
            swerve.drive(0, -.01, 0, false, true, false);
        } else {
            swerve.drive(0, -15, 0, false, true, false);

        }
    }


    @Override 
    public boolean isFinished ( ) { 
        return (timer.hasElapsed(.95));
    }
}
