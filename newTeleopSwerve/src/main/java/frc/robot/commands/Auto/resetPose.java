package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.LimelightHelpers;

public class resetPose extends InstantCommand {
    private final SwerveSubsystem swerve;

    public resetPose(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetPose(new Pose2d());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
