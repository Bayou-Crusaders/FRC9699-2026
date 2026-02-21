package frc.robot;

import java.util.Collection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


/*  A class for utility functions that can be used across multiple files*/
public class Util {

    public static Collection<Pose2d> shootToHubPoses = null;
    
    public static double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) { // Calculate the distance between two poses
        Translation2d translation1 = pose1.getTranslation();
        Translation2d translation2 = pose2.getTranslation();
        return translation1.getDistance(translation2);
    }

}
