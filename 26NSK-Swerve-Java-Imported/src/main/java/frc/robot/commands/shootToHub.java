package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Util;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;

public class shootToHub extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Shooter shooter;
    private Targeting targeting;
    private Translation2d targetHubTranslation;
    private Pose2d nearestShootingPose;
    
    public shootToHub(CommandSwerveDrivetrain drivetrain, Shooter shooter, Targeting targeting) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.targeting = targeting;
    }

    @Override
    public void initialize() {
        // Get the robot's current pose
        Pose2d robotPose = drivetrain.getState().Pose;

        // Find the nearest shooting pose to the robot's current pose based on the alliance
        nearestShootingPose = (Util.getAlliance() == Alliance.Blue)
                ? robotPose.nearest(Util.shootToBlueHubPoses)
                : robotPose.nearest(Util.shootToRedHubPoses);

        // Find the Translation2d of the hub based on the alliance
        targetHubTranslation = (Util.getAlliance() == Alliance.Blue)
                ? Util.blueHubPose.getTranslation()
                : Util.redHubPose.getTranslation();

        // Drive to the nearest shooting pose
        drivetrain.pathfindToPose(nearestShootingPose);
    }

    @Override
    public void execute() {
        
        // Once at the shooting pose, calculate the distance to the target and the angle to the target
        if (drivetrain.getState().Pose.getTranslation().getDistance(nearestShootingPose.getTranslation()) < 0.1) { // If the robot is within 10 cm of the shooting pose
            double distanceToTarget = targeting.distanceFromHub(drivetrain.getState().Pose);
            double angleToTarget = targeting.angleToTarget(drivetrain.getState().Pose, targetHubTranslation);

            // Use the distance and angle to set the shooter speed and angle
            shooter.setFlywheelRPM(distanceToTarget);
            shooter.setPitchAngle(angleToTarget);
        } else {
            // do nothing while driving towards the shooting pose
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the shooter when the command ends
        shooter.setFlywheelRPM(0);
    }

}
