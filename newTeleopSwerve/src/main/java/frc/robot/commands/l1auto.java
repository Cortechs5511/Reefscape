package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class l1auto extends Command {
    private final SwerveSubsystem m_swerve;
    private final CoralSubsystem m_coral;
    private final Elevator m_elevator;

    private final Timer timer = new Timer();

    public l1auto(SwerveSubsystem swerve, CoralSubsystem coral, Elevator elevator) {
        this.m_swerve = swerve;
        this.m_coral = coral;
        this.m_elevator = elevator;
        addRequirements(swerve, coral, elevator);
    }

    // helper limelight functions
    private boolean checkLimeLight() {
        double id = 0;

        RawFiducial[] limelightData  = m_swerve.getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            double currentTa = currentEntry.ta;
            double currentTx = currentEntry.txnc;
            id = currentEntry.id;
        }

        if ( id == 0 ) {
            return false;
        }

        return true;
    }

    private boolean checkAlignment () { 
        double currentTa = 0 ; 
        double currentTx = 0 ;
        double id = 0 ; 

        RawFiducial[] limelightData  = m_swerve.getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            currentTa = currentEntry.ta;
            currentTx = currentEntry.txnc;
            id = currentEntry.id;
        }

        if ( id == 0 ) {
            return false;
        }

        if ((currentTa >= .1 && currentTa <= .15) && (currentTx >= 1.6 && currentTx <= 2.9)) { 
            return true;
        }
        
        return false;
    }

    @Override
    public void initialize() {
        timer.restart();
    }


    @Override
    public void execute() {

        m_coral.setWristPos(0.405);


        // if (!timer.hasElapsed(.75)) { 
        //     m_coral.setWristPos(0.405);
        // } else if (timer.hasElapsed(.75) && !timer.hasElapsed(4)){ 
        //     m_swerve.drive(10, 0, 0, false, true, false);
        // } else if (timer.hasElapsed(5) && !timer.hasElapsed(5.3)) { 
        //     m_swerve.drive(-0.001, 0, 0, false, true, false);
        // } else if (timer.hasElapsed(5.3) && !timer.hasElapsed(8.3)) { 
        //     m_swerve.drive(-.05, 0, 0, false, true, false);
        // } else if (timer.hasElapsed(8.3) && !timer.hasElapsed(8.6)) {
        //     m_swerve.drive(0, -0.001, 0, false, true, false);
        // } else if (timer.hasElapsed(8.6) && !timer.hasElapsed(11)) {
        //     if (checkLimeLight()) {
        //         m_swerve.drive(0, m_swerve.limelightAlignStrafe(), 0, false, true, false);
        //     } else {
        //         m_swerve.drive(0, 0, 0, false, true, false);
        //     }
        // } else if (timer.hasElapsed(11) && !timer.hasElapsed(13)) { 
        //     if (checkLimeLight()) { 
        //         m_swerve.drive(m_swerve.limelightAlignDrive(), 0, 0, false, true, false);
        //         m_swerve.drive(0, 0, 0, false, true, false);
        //     }
        // } else if (timer.hasElapsed(13) && !timer.hasElapsed(14)) {
        //     m_coral.setWristPos(0.595);
        // } else if (timer.hasElapsed(14.5)) {
        //     m_coral.setFlywheelPower(0, .6);
        // }
                
    }
}
