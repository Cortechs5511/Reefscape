package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;

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
        currentPose = swerve.getPose(); 
        targetPose = currentPose.plus(new Transform2d(0.0, LimelightConstants.RIGHT_REEF_TRANSLATION, new Rotation2d())); // add 0.05 meters to the right
        
        driveController.setSetpoint(targetPose.getTranslation().getY());
        driveController.setTolerance(LimelightConstants.Y_TOLERANCE_REEF_ALIGNMENT);
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        double ySpeed = driveController.calculate(currentPose.getTranslation().getY());

        swerve.drive(0.0, ySpeed, 0.0, false, false,false);
    }

}
