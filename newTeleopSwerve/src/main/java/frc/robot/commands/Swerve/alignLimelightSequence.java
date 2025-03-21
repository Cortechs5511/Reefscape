package frc.robot.commands.Swerve;
 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class alignLimelightSequence extends SequentialCommandGroup {
    public alignLimelightSequence(SwerveSubsystem swerve) {
        addCommands(
            new alignLimelightAngle(swerve),
            new alignLimelight(swerve),
            new alignLimelightDist(swerve)
        );
    }
}
 