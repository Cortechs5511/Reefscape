package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {
    private Pigeon2 pigeon;

     public Gyro() {
         pigeon = new Pigeon2(0);  
     }

     public Rotation2d getRotation2d() {
         // Get yaw in degrees from the pigeon
         // had to add an offset because it was off by 45°
         double yawDegrees = pigeon.getYaw().getValueAsDouble()-45+135; 
         
         // Convert degrees to radians (Rotation2d uses radians)
         double yawRadians = Math.toRadians(yawDegrees);
         
         // Create and return a Rotation2d object
         return new Rotation2d(yawRadians);
     }

     public void resetGyro() {
         // pigeon.reset();
         pigeon.setYaw(0);
     }

 }
