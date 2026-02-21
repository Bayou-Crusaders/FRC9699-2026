
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class Targeting implements Subsystem {
    private Translation2d target = new Translation2d();
    private InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

    public Targeting() {
        // Use the following InterpolatingDoubleTreeMaps to map distance to RPM and distance to angle for shooting
        //rpmMap.put(null, null); // Distance (m) to RPM (RPM) mapping
        //rpmMap.put(null, null); 
        //rpmMap.put(null, null); 

        //angleMap.put(null, null); // Distance (m) to angle (degrees) mapping
        //angleMap.put(null, null); 
        //angleMap.put(null, null); 
    }

    // Move the target pose by the specified x and y offsets
    public void moveTargetPose(double x, double y) { 
        target = target.plus(new Translation2d(x, y));
    }
    
    // Get the current target pose as a Translation2d
    public Translation2d getTargetPose() { 
        return new Translation2d(target.getX(), target.getY());
    }
    
    // Get the current target pose as an array of x and y coordinates
    public double[] getTargetXY() { 
        return new double[] {target.getX(), target.getY()};
    }

    // Calculate the distance from the robot's current pose to the target pose
    public Double distanceToTarget(Pose2d RobotPose) { 
        Translation2d robotPose = RobotPose.getTranslation();
        Translation2d targetPose = this.getTargetPose();
        return robotPose.getDistance(targetPose);
    }

    public Double distanceFromHub(Pose2d RobotPose) {
        return RobotPose.getTranslation().getDistance(new Translation2d(0, 0)); // TODO: Find Hub Position
    }

    // Get the RPM for shooting based on the distance to the target using interpolation
    public Double getRPMForDistance(double distance) { 
        return rpmMap.get(distance);
    }

    // Get the angle for shooting based on the distance to the target using interpolation
    public Double getAngleForDistance(double distance) { 
        return angleMap.get(distance);
    }
}