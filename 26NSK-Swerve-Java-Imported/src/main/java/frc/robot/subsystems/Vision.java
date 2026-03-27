package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final EstimateConsumer estConsumer;
    private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private final Transform3d kRobotToCam = new Transform3d(0.289, 0.117, 0.193, new Rotation3d(0, 0, Units.degreesToRadians(90)));
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera("leftCamera");
        photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(kTagLayout);
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, new Rotation2d(70));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            visionSim.addCamera(cameraSim, kRobotToCam);
            cameraSim.enableDrawWireframe(true);
        }
    }

    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(est -> getSimDebugField().getObject("VisionEstimation").setPose(est), null);
            }
        }
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
