package frc.robot.commands.Auto;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class resetGyro extends Command {
    private final SwerveSubsystem m_swerve; 
    private double gyroOffsetAngle;

    public resetGyro (SwerveSubsystem swerve, double gyroOffsetAngle) { 
        m_swerve = swerve;
        this.gyroOffsetAngle = gyroOffsetAngle;
    }

    @Override
    public void initialize() {
        m_swerve.resetGyro(gyroOffsetAngle);
    }

    @Override
    public boolean isFinished() {
        return (true);
    }
    

}
