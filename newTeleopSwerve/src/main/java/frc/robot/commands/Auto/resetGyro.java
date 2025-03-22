package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.Coral.setWristPosition;
import frc.robot.commands.Swerve.AlignToReefTagRelative;
import frc.robot.commands.Swerve.alignLimelight;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Swerve.Gyro;

public class resetGyro extends Command {
    private final SwerveSubsystem m_swerve;
    private double angle = 0 ; 
    private boolean gyroIsReset = false; 


    private final Timer timer = new Timer();

    public resetGyro(SwerveSubsystem swerve, double offset) {
        this.m_swerve = swerve;
        this.angle = offset;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        gyroIsReset = false; 
    }


    @Override
    public void execute() {
        if (timer.hasElapsed(.1) && !gyroIsReset) { 
            m_swerve.resetGyro(angle);
            gyroIsReset = true;
        }
    }

    @Override
    public boolean isFinished() {
        return gyroIsReset;
    }
}
