// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AlignToReefAuto extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer, cutoffTimer;
  private SwerveSubsystem swerve;
  private double tagID = -1;

  public AlignToReefAuto(boolean isRightScore, SwerveSubsystem subsystem) {
    xController = new PIDController(LimelightConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(LimelightConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(LimelightConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    swerve = subsystem; 
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    this.cutoffTimer = new Timer();
    this.cutoffTimer.start();

    rotController.setSetpoint(LimelightConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(LimelightConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(LimelightConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(LimelightConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(LimelightConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(LimelightConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-sublime");
  }

  @Override
  public void execute() {

    if (LimelightHelpers.getTV("limelight-sublime") && LimelightHelpers.getFiducialID("limelight-sublime") == tagID) {

      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-sublime");
      SmartDashboard.putNumber("test/x pos", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("test/xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      SmartDashboard.putNumber("test/y pos", postions[0]);
      double rotValue = -rotController.calculate(postions[4]);
      SmartDashboard.putNumber("test/rot pos", postions[4]);

      swerve.drive(xSpeed * 35 , ySpeed * 35, rotValue, true, true, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      swerve.drive(0,0, 0, false, false, false);
    }

    SmartDashboard.putNumber("test/poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive( 0, 0, 0, false, false, false);
  }

  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(LimelightConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(LimelightConstants.POSE_VALIDATION_TIME) || cutoffTimer.hasElapsed(LimelightConstants.CUTOFF_TIME_AUTO);
  }
}