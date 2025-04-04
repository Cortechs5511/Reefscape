package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.LimelightHelpers;

public class RelocalizeWithLimelight extends InstantCommand {
    private final SwerveSubsystem swerve;

    public RelocalizeWithLimelight(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // get the current Limelight pose
        double[] pose = null;
        pose = LimelightHelpers.getBotPose_wpiBlue("limelight-sublime");

        if (pose != null && pose.length >= 6) {
            double x = pose[0];  
            double y = pose[1];
            double heading = pose[5]; 

            // Create a Pose2d object and set the new pose in odometry
            Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
            swerve.resetPose(newPose);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
