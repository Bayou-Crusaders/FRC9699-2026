
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Util;


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
        return RobotPose.getTranslation().getDistance(this.getTargetPose());
    }

    public Double angleToTarget(Pose2d RobotPose) {
        Translation2d robotTranslation = RobotPose.getTranslation();
        Translation2d targetTranslation = this.getTargetPose();
        double angleToTarget = Math.atan2(targetTranslation.getY() - robotTranslation.getY(), targetTranslation.getX() - robotTranslation.getX());
        return Math.toDegrees(angleToTarget);
    }

    public Double angleToTarget(Pose2d RobotPose, Translation2d targetIn) {
        Translation2d robotTranslation = RobotPose.getTranslation();
        double angleToTarget = Math.atan2(targetIn.getY() - robotTranslation.getY(), targetIn.getX() - robotTranslation.getX());
        return Math.toDegrees(angleToTarget);
    }

    public Double distanceFromHub(Pose2d RobotPose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return RobotPose.getTranslation().getDistance(Util.redHubPose.getTranslation());
        } else {
            return RobotPose.getTranslation().getDistance(Util.blueHubPose.getTranslation());
        }
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