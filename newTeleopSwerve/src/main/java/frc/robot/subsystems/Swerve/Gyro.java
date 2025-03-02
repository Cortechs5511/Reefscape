package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase{
    private Pigeon2 pigeon;

     public Gyro() {
         pigeon = new Pigeon2(0);  
     }

     public Rotation2d getRotation2d() {
         // Get yaw in degrees from the pigeon
         // had to add an offset because it was off by 45°
         double yawDegrees = pigeon.getYaw().getValueAsDouble()-45; 
         
         // Convert degrees to radians (Rotation2d uses radians)
         double yawRadians = Math.toRadians(yawDegrees);
         
         // Create and return a Rotation2d object
         return new Rotation2d(yawRadians);
     }

     public void resetGyro() {
         // pigeon.reset();
         pigeon.setYaw(0);
     }

     @Override
     public void periodic() {
        SmartDashboard.putNumber("Gyro/Yaw Radians per Sec", Math.toRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble()));
     }

 }
