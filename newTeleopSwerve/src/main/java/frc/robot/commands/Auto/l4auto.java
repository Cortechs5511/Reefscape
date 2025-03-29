package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class l4auto extends Command {
    private final SwerveSubsystem m_swerve;
    private final CoralSubsystem m_coral;
    private final Elevator m_elevator;


    private final Timer timer = new Timer();

    public l4auto(SwerveSubsystem swerve, CoralSubsystem coral, Elevator elevator) {
        this.m_swerve = swerve;
        this.m_coral = coral;
        this.m_elevator = elevator;
        addRequirements(swerve, coral, elevator);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        m_coral.setWristPosPID(.56);
        m_elevator.setPositionPID(3.55);

        if (timer.hasElapsed(4) && !timer.hasElapsed((4.5))) {
            m_coral.setFlywheelPower(0, 1.0);
        } else if (timer.hasElapsed(4.5)) { 
            m_coral.setFlywheelPower(0, 0);
            m_coral.setWristPosPID (.405);
            m_elevator.setPositionPID(0);
        }
        
        
    }
}
