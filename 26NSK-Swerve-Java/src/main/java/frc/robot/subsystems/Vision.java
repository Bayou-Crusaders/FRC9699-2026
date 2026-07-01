package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Vision {
    
    public static final AprilTagFieldLayout kTagLayout = 
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(-0.261, 0.235, 0.293), 
        new Rotation3d(355, 0, 90)
    );

    private PhotonCamera camera = new PhotonCamera("Left Hopper");
    
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCamera);

    private Optional<EstimatedRobotPose> visionEst = Optional.empty();

    private CommandSwerveDrivetrain kdrivetrain;
    
    public Vision(CommandSwerveDrivetrain drivetrain) {
        kdrivetrain = drivetrain;
    }

    public void applyEstimatedPose() {
        if (!camera.getAllUnreadResults().isEmpty()) {
            for (var result : camera.getAllUnreadResults()) {
                visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
                }
                kdrivetrain.addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), visionEst.get().timestampSeconds);
            }
        } else {
            visionEst = Optional.empty();
        }
    }

    public Boolean hasNewPose() {
        return visionEst.isPresent();
    }
}