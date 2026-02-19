
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

    public void moveTargetPose(double x, double y) { // Move the target pose by the specified x and y offsets
        target = target.plus(new Translation2d(x, y));
    }

    public Translation2d getTargetPose() { // Get the current target pose as a Translation2d
        return new Translation2d(target.getX(), target.getY());
    }

    public double[] getTargetXY() { // Get the current target pose as an array of x and y coordinates
        return new double[] {target.getX(), target.getY()};
    }

    public Double distanceToTarget(Pose2d RobotPose) { // Calculate the distance from the robot's current pose to the target pose
        Translation2d robotPose = RobotPose.getTranslation();
        Translation2d targetPose = this.getTargetPose();
        return robotPose.getDistance(targetPose);
    }
}