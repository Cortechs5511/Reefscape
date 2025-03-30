package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private final PIDController[] drivePIDControllers;
    private final ProfiledPIDController[] turnPIDControllers;
    // private final PIDController[] turnPIDControllers;

    // Properties for Field oriented driving
    private Gyro fieldGyro;
    
    // for resetting gyro
    private Rotation2d gyroOffset = new Rotation2d();

    private SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();

    private XboxController driveController;

    public SwerveSubsystem() {
        fieldGyro = new Gyro();
        gyroOffset = new Rotation2d();
        // Initialize the swerve modules
        modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.IDS[0], SwerveConstants.IDS[1], SwerveConstants.IDS[2]),
            new SwerveModule(SwerveConstants.IDS[3], SwerveConstants.IDS[4], SwerveConstants.IDS[5]),
            new SwerveModule(SwerveConstants.IDS[6], SwerveConstants.IDS[7], SwerveConstants.IDS[8]),
            new SwerveModule(SwerveConstants.IDS[9], SwerveConstants.IDS[10], SwerveConstants.IDS[11]),
        };
        // Initalize kinematic objects
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[0], SwerveConstants.MODULE_TRANSLATIONS[1]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[2], SwerveConstants.MODULE_TRANSLATIONS[3]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[4], SwerveConstants.MODULE_TRANSLATIONS[5]),
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[6], SwerveConstants.MODULE_TRANSLATIONS[7])
        );
        // initialize PIDidontrollers 
        drivePIDControllers = new PIDController[] {
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
            new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2])
        };
        turnPIDControllers = new ProfiledPIDController[] {
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION)),
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION)),
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION)),
             new ProfiledPIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2], new TrapezoidProfile.Constraints(SwerveConstants.ANGLE_MAX_VELOCITY, SwerveConstants.ANGLE_MAX_ACCELERATION))  
        };
        for (ProfiledPIDController p : turnPIDControllers) {
            p.enableContinuousInput(0, 2 * Math.PI);
        };

        // For Field driving 
        odometry = new SwerveDriveOdometry(kinematics, fieldGyro.getRotation2d(), getPositions());
        SmartDashboard.putData("Field", field);

        driveController = new XboxController(0);

    
        RobotConfig config;
        try{

        config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(
                speeds.vxMetersPerSecond / SwerveConstants.MAX_TRANSLATIONAL_SPEED,
                speeds.vyMetersPerSecond / SwerveConstants.MAX_TRANSLATIONAL_SPEED,
                speeds.omegaRadiansPerSecond / SwerveConstants.MAX_ROTATIONAL_SPEED,
                false,  // fieldRelative (assuming you want robot relative in auto)
                false,  // alignLimelight
                false   // resetGyro
            ), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]), // Translation PID constants
                        new PIDConstants(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2]) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
    }

        // get limelight data
        public RawFiducial[] getLimelightData() {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-sublime");
        return fiducials;
        }

        public boolean detectLimelight () { 
            return LimelightHelpers.getTV("limelight-sublime");
        }

        public double getDistanceFromTag () {
            double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-sublime");
            System.out.println(postions[2]);
            return postions[2];
        }

        public void resetGyro(double degrees) { 
            fieldGyro.resetGyro(degrees);
        }

    @Override 
    public void periodic () {
        double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-sublime");
        SmartDashboard.putNumber("limelight/x pos", postions[2]);
        SmartDashboard.putNumber("limelight/y pos", postions[0]);
        SmartDashboard.putNumber("limelight/rot pos", postions[4]);

        // pose stuff 
        odometry.update(fieldGyro.getRotation2d(), getPositions());
        field.setRobotPose(getPose());
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Pose X", pose.getX());
        SmartDashboard.putNumber("Pose Y", pose.getY());
        SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());

        // transfering the controller inputs into SwerveModuleState
        ChassisSpeeds controllerSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * -driveController.getLeftY(), 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * -driveController.getLeftX(), 
            SwerveConstants.MAX_ROTATIONAL_SPEED * -driveController.getRightX()
        );
        SwerveModuleState[] controllerStates = kinematics.toSwerveModuleStates(controllerSpeeds);
        
        // Logging the swerve Module state data onto the smart module
        double[] controllerStatesAsDoubles = {
            controllerStates[0].angle.getRadians(),
            controllerStates[0].speedMetersPerSecond,
            controllerStates[1].angle.getRadians(),
            controllerStates[1].speedMetersPerSecond,
            controllerStates[2].angle.getRadians(),
            controllerStates[2].speedMetersPerSecond,
            controllerStates[3].angle.getRadians(),
            controllerStates[3].speedMetersPerSecond,
        };
        SmartDashboard.putNumberArray("Controller State", controllerStatesAsDoubles);

        logStates(); 
    }


    public void drive(double y, double x, double theta, boolean fieldRelative, boolean alignLimelight, boolean resetGyro) {
        ChassisSpeeds newDesiredSpeeds; 
        
        if (alignLimelight) { 
            newDesiredSpeeds = new ChassisSpeeds(y, x, theta);
        } else { 
            newDesiredSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * y, 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * x,
            SwerveConstants.MAX_ROTATIONAL_SPEED * theta
        );
        }

        // reset gyro button
        if (resetGyro) {
            gyroOffset = fieldGyro.getRotation2d();
        }

        // implementing field logic
        if (fieldRelative) {
            driveFieldRelative(newDesiredSpeeds);
        }
        else {
            driveRobotRelative(newDesiredSpeeds);
        }

        // driveRobotRelative(newDesiredSpeeds);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        SmartDashboard.putNumber("Swerve/SwerveModule distance", modules[0].getPosition().distanceMeters );
        SmartDashboard.putNumber("Swerve/SwerveModule degree", modules[0].getPosition().angle.getDegrees() );
        return positions;
    } 

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(fieldGyro.getRotation2d(), getPositions(), pose);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        Rotation2d drivingAngle = fieldGyro.getRotation2d().minus(gyroOffset);
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drivingAngle));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        // add constant for max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_TRANSLATIONAL_SPEED);

        modules[0].setTargetState(targetStates[0], drivePIDControllers[0], turnPIDControllers[0]);
        modules[1].setTargetState(targetStates[1], drivePIDControllers[1], turnPIDControllers[1]);
        modules[2].setTargetState(targetStates[2], drivePIDControllers[2], turnPIDControllers[2]);
        modules[3].setTargetState(targetStates[3], drivePIDControllers[3], turnPIDControllers[3]);
    //    for (int i = 0; i<4; i++) {
    //     modules[i].setTargetState(targetStates[i], drivePIDControllers[i], turnPIDControllers[i]);
    //    }
    }


    public SwerveModuleState[] getStates() {
        SwerveModuleState[] currentStates = {
            modules[0].getState(), 
            modules[1].getState(), 
            modules[2].getState(), 
            modules[3].getState(), 
        };
        return currentStates;
    }

    public void logStates() {
        double[] loggingState = {
            modules[0].getAngle().getRadians(),
            modules[0].getVelocity(),
            modules[1].getAngle().getRadians(),
            modules[1].getVelocity(),
            modules[2].getAngle().getRadians(),
            modules[2].getVelocity(),
            modules[3].getAngle().getRadians(),
            modules[3].getVelocity(),
        };
        
        SmartDashboard.putNumberArray("Module State", loggingState);
        SmartDashboard.putNumber("Gyro Radians", fieldGyro.getRotation2d().getRadians());

        SmartDashboard.putNumber("Swerve/FL Velocity", modules[0].getVelocity());
        SmartDashboard.putNumber("Swerve/FR Velocity", modules[1].getVelocity());
        SmartDashboard.putNumber("Swerve/BL Velocity", modules[2].getVelocity());
        SmartDashboard.putNumber("Swerve/BR Velocity", modules[3].getVelocity());
    }
    
}