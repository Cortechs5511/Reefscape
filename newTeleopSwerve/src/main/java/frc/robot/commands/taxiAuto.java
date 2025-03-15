package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class taxiAuto extends Command {
    private final SwerveSubsystem m_swerve;
    private final CoralSubsystem m_coral;
    private final Elevator m_elevator;

    private final Timer timer = new Timer();

    public taxiAuto(SwerveSubsystem swerve, CoralSubsystem coral, Elevator elevator) {
        this.m_swerve = swerve;
        this.m_coral = coral;
        this.m_elevator = elevator;
        addRequirements(swerve, coral, elevator);
        }


    @Override
    public void initialize() {
        timer.restart();
    }


    @Override
    public void execute() {
        if (!timer.hasElapsed(1)) { 
            m_swerve.drive(.001, 0, 0, false, false, false);
            m_coral.setWristPos(0.405);
        } else if (!timer.hasElapsed(10)){ 
            m_coral.setWristPos(0.405);
            m_swerve.drive(5, 0, 0, false, true, false);
        }
    }

}