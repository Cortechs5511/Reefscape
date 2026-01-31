package frc.robot.commands.Auto;

import java.security.AlgorithmParameterGenerator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class algifyBottom extends Command {
    private final SwerveSubsystem m_swerve;
    private final CoralSubsystem m_coral;
    private final Elevator m_elevator;
    private final AlgaeSubsystem m_algae; 
    private final PIDController driveController; 
    private Pose2d targetPose2d;
    private Pose2d currentPose2d;


    private final Timer timer = new Timer();

    public algifyBottom(SwerveSubsystem swerve, CoralSubsystem coral, Elevator elevator, AlgaeSubsystem algae) {
        this.m_swerve = swerve;
        this.m_coral = coral;
        this.m_elevator = elevator;
        this.m_algae = algae; 
        this.driveController = new PIDController(LimelightConstants.X_REEF_ALIGNMENT_P, 0, 0);
        addRequirements(swerve, coral, elevator);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        m_swerve.resetPose(new Pose2d());

        currentPose2d = m_swerve.getPose(); 
        targetPose2d = currentPose2d.plus(new Transform2d(-.75 , 0.0, new Rotation2d().rotateBy(currentPose2d.getRotation())));
        driveController.setSetpoint(targetPose2d.getX());
        driveController.setTolerance(.05);
    }

    @Override
    public void execute() {
        m_coral.setWristPosPID(.45);
        m_elevator.setPositionPID(2.8);

        if (timer.hasElapsed(2.5) && !timer.hasElapsed((3.0))) {
            m_algae.setWristPower(-AlgaeConstants.WRIST_MAX_POWER);
            m_algae.setFlywheelPower(1.0);
        } else if (timer.hasElapsed(3.5)) {
            double driveBackSpeed = driveController.calculate(m_swerve.getPose().getX());
            m_swerve.drive(driveBackSpeed * 15, 0, 0, false, true, false);
            m_coral.setFlywheelPower(0, 0);
            m_coral.setWristPosPID (.33);
            m_elevator.setPositionPID(0);
            m_algae.setFlywheelPower(0.0);
            m_algae.setWristPosPID(0);
        }
        
    }

    @Override
    public boolean isFinished() {
        return (m_elevator.getAccumulatedRotations() < .3 && timer.hasElapsed(3.25));
    }
}
