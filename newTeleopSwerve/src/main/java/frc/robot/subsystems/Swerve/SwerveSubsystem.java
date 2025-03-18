package frc.robot.subsystems.Swerve;

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
import edu.wpi.first.wpilibj.Timer;
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
    private Gyro gyro;
    private SwerveDriveOdometry odometry;
    private Field2d field;

    private XboxController driveController;

    public SwerveSubsystem() {
        gyro = new Gyro();
        // Initialize the swerve modulesi
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

        // turnPIDControllers = new PIDController[] {
        //     new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
        //     new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
        //     new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]),
        //     new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2])
        // };

        for (ProfiledPIDController p : turnPIDControllers) {
            p.enableContinuousInput(0, 2 * Math.PI);
        };

        // For Field driving later 
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
        field = new Field2d();

        driveController = new XboxController(0);

    
        RobotConfig config;
        try{

        config = RobotConfig.fromGUISettings();

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this:: driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
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

    private double getDesiredYawForTag(int tagID) {
        switch (tagID) { 
            // case 6:   
            case 7:
            case 18:
                return Math.toRadians(0);
    
            case 6:
            case 19:
                return Math.toRadians(60);
    
            case 11:
            case 20:
                return Math.toRadians(120);
    
            case 10:
            case 21:
                return Math.toRadians(180);
    
            case 9:
            case 22:
                return Math.toRadians(-120);
    
            case 8:
            case 17:
                return Math.toRadians(-60);
    
            default:
                return Math.toRadians(0);
        }
    }

    // limelight stuff
    public RawFiducial[] getLimelightData() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-sublime");
    return fiducials;
    }

    public double alignAngle() {
        RawFiducial[] limelightData = getLimelightData();
        if (limelightData.length == 0) {
            return 0.0;
        }
    
        int tagID = limelightData[limelightData.length-1].id;
    
        double desiredYaw = getDesiredYawForTag(tagID); 
    
        double currentYaw = gyro.getRotation2d().getRadians();
        
        double rotationCmd = turnPIDControllers[0].calculate(currentYaw, desiredYaw);
    
        rotationCmd = Math.min(rotationCmd, 2.0); 
        rotationCmd = Math.max(rotationCmd, -2.0); 
    
        return rotationCmd;
    }

    public double limelightAlignStrafe() {
        double currentTx = 0 ; 
        RawFiducial[] limelightData  = getLimelightData();

        if (limelightData.length == 0) {
            return 0;
        }
    
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            currentTx = currentEntry.txnc;
        }

        double strafeSpeed = .8 * -(currentTx - 2.3);
        
        // 1.8 to 2.9
        if (2.1 <= currentTx && currentTx <= 2.6)  {
            return 0.001; 
    }


        if (Math.abs(strafeSpeed) <= 2) {
            if (strafeSpeed > 0) {
                // strafeSpeed = (currentTx <= 13.2) ? 1 : 2;
                strafeSpeed = 2;
            } else {
                // strafeSpeed = (currentTx >= 10.2) ? -1 : -2;
                strafeSpeed = -2;
            }
        }
    
        return strafeSpeed;
    }

    public double limelightAlignDrive() {
        double speed = -5;
        double currentTa = 0; 
        int id = 0; 
        // double limelightMountAngleDegrees = 0.0; 
        // double limelightLensHeightInches = 8;
        // double goalHeightInches = 9.5; 

        RawFiducial[] limelightData  = getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            currentTa = currentEntry.ta;
            id = currentEntry.id;
        }

        if (id == 0) {
            return 0;
        }

        speed = 0.75 *  100 * -(currentTa - .15);
        // not sure best value for ta right now 
        if (currentTa >= .145 && currentTa <= 0.17) {
            return 0;
        }
    
        if (Math.abs(speed) <= 2) {
            if (speed > 0) {
                // speed = (currentTx <= 13.2) ? 1 : 2;
                speed = 2;
            } else {
                // speed = (currentTx >= 10.2) ? -1 : -2;
                speed = -2;
            }
        }
       
        return speed;
    }

    public boolean txIsAligned() { 
        double currentTx = 0 ; 
        RawFiducial[] limelightData  = getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            currentTx = currentEntry.txnc;
        }   
        return (currentTx > 1.7 && currentTx < 2.9);
    }

    public boolean taIsAligned() { 
        double currentTa = 0 ; 
        RawFiducial[] limelightData  = getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            currentTa = currentEntry.ta;
        }   
        return (currentTa > .145 && currentTa < .19);
    }

    public boolean angleIsAligned() {
        int id = 0;
        RawFiducial[] limelightData  = getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            id = currentEntry.id;
        }   

        double desiredAngle = getDesiredYawForTag(id);

        return Math.abs(desiredAngle - gyro.getRotation2d().getRadians()) <= 0.087;
    }

    @Override 
    public void periodic () {
        double currentTa = 0 ; 
        double currentTx = 0 ;
        RawFiducial[] limelightData  = getLimelightData();
        for (int i = 0; i < limelightData.length; i++) {
            RawFiducial currentEntry = limelightData[i];
            currentTa = currentEntry.ta;
            currentTx = currentEntry.txnc;
        }   
        SmartDashboard.putNumber("Limelight/TX", currentTx);
        SmartDashboard.putBoolean("Limelight/TX Aligned", (currentTx > 1.7 && currentTx < 2.9));
        SmartDashboard.putNumber("Limelight/TA", currentTa);
        SmartDashboard.putBoolean("Limelight/TA Aligned", (currentTa > .145 && currentTa < .19));

        if  (!(currentTa > .249 && currentTa < .305)) { 
            if (currentTa < .249) {
                SmartDashboard.putString("Limelight/TA F or B", "Forwards");
            } else if (currentTa > .305) {
                SmartDashboard.putString("Limelight/TA F or B", "Backwards");
            }
        } else {
            SmartDashboard.putString("Limelight/TA F or B", "GOOD");
        }
        

        SmartDashboard.putString("Limelight/TA Aligned", ((currentTa < .249) ? "Forward" : "backwards"));


        


        odometry.update(gyro.getRotation2d(), getPositions());
        field.setRobotPose(getPose());
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
            gyro.resetGyro();
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

        return positions;
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
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
        SmartDashboard.putNumber("Gyro Radians", gyro.getRotation2d().getRadians());

        SmartDashboard.putNumber("Swerve/FL Velocity", modules[0].getVelocity());
        SmartDashboard.putNumber("Swerve/FR Velocity", modules[1].getVelocity());
        SmartDashboard.putNumber("Swerve/BL Velocity", modules[2].getVelocity());
        SmartDashboard.putNumber("Swerve/BR Velocity", modules[3].getVelocity());
    }
    
}