package frc.robot;

import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/*  A class for utility functions that can be used across multiple files*/
public class Util {
    //TODO: Update these poses to be the actual shooting poses for the blue and red hubs
    public static Collection<Pose2d> shootToBlueHubPoses = List.of(
        new Pose2d(1.0, 1.0, new Rotation2d(0.0)),
        new Pose2d(2.0, 2.0, new Rotation2d(0.0))
    );

    public static Collection<Pose2d> shootToRedHubPoses = List.of(
        new Pose2d(1.0, 1.0, new Rotation2d(0.0)),
        new Pose2d(2.0, 2.0, new Rotation2d(0.0))
    );

    public static Pose2d blueHubPose = new Pose2d(4.623, 4.030, new Rotation2d(0)); // The position of the blue hub on the field
    public static Pose2d redHubPose = new Pose2d(11.913, 4.030, new Rotation2d(0)); // The position of the red hub on the field
    
    public static double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) { // Calculate the distance between two poses
        Translation2d translation1 = pose1.getTranslation();
        Translation2d translation2 = pose2.getTranslation();
        return translation1.getDistance(translation2);
    }

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

}
