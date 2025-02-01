
package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyro {
    private AHRS navX;

    public Gyro() {
        navX = new AHRS(NavXComType.kMXP_SPI);  
    }

    public Rotation2d getRotation2d() {
        // Get yaw in degrees from the navX
        double yawDegrees = -1 * navX.getYaw() - 135; 
        
        // Convert degrees to radians (Rotation2d uses radians)
        double yawRadians = Math.toRadians(yawDegrees);
        
        // Create and return a Rotation2d object
        return new Rotation2d(yawRadians);
    }

    public void resetGyro() {
        navX.reset();
    }

}