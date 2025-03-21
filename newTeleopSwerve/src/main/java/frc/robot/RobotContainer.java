// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root dirdriveectory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto.l1auto;
import frc.robot.commands.Auto.taxiAuto;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Coral.setCoralPower;
import frc.robot.commands.Coral.setWristPosition;
import frc.robot.commands.Elevator.setElevatorPosition;
import frc.robot.commands.Elevator.setElevatorPower;
import frc.robot.commands.Swerve.AlignToReefTagRelative;
import frc.robot.commands.Swerve.alignLimelight;
import frc.robot.commands.Swerve.alignLimelightAngle;
import frc.robot.commands.Swerve.alignLimelightDist;
import frc.robot.commands.Swerve.swerveDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final Elevator m_elevator = new Elevator();
  private final CoralSubsystem m_coral = new CoralSubsystem();

  private final OI oi = OI.getInstance();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("taxi", new SequentialCommandGroup (new taxiAuto (m_swerveSubsystem, m_coral, m_elevator)));
    autoChooser.addOption("l2", new SequentialCommandGroup (new l1auto (m_swerveSubsystem, m_coral, m_elevator)));

    SmartDashboard.putData("Auto chooser", autoChooser);
    // Configure the trigger bindings
    m_swerveSubsystem.setDefaultCommand(new swerveDrive(m_swerveSubsystem));
    m_elevator.setDefaultCommand(new setElevatorPower(m_elevator));
    m_coral.setDefaultCommand(new setCoralPower(m_coral));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release. 
    
    // limelight stuff
    m_driverController.a().whileTrue(new alignLimelight(m_swerveSubsystem));
    m_driverController.b().whileTrue(new alignLimelightDist(m_swerveSubsystem));
    m_driverController.y().whileTrue(new alignLimelightAngle(m_swerveSubsystem));
    // m_driverController.y().whileTrue(new AlignToReefTagRelative(true, m_swerveSubsystem));


    // driving position (bottom) 
    m_operatorController.a().whileTrue(new setWristPosition(m_coral, 0.405)).whileTrue(new setElevatorPosition(m_elevator, 0, false));
    // l2
    m_operatorController.x().whileTrue(new setWristPosition(m_coral, 0.6)).whileTrue(new setElevatorPosition(m_elevator, 0, false));
    // l3
    m_operatorController.b().whileTrue(new setWristPosition(m_coral, 0.625)).whileTrue(new setElevatorPosition(m_elevator, 1.8, false));
    // l4 wrist
    m_operatorController.y().whileTrue(new setWristPosition(m_coral, 0.55));
    // l4 elevator
    m_operatorController.leftStick().whileTrue(new setElevatorPosition(m_elevator, 3.55, false));
    // intake
    m_operatorController.leftBumper().whileTrue(new setWristPosition(m_coral, 0.55)).whileTrue(new setElevatorPosition(m_elevator, 0, false));
}

  /**\[]\[]
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}