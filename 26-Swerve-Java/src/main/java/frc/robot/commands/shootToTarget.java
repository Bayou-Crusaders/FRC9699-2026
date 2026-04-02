package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Targeting;
import frc.robot.Util;

public class shootToTarget extends Command {
    CommandSwerveDrivetrain m_drivetrain;
    Shooter m_shooter;
    Targeting m_targeting;
    Double angleToRotate;
    Pose2d referencePose;
    Pose2d robotPose;

    public shootToTarget(
        CommandSwerveDrivetrain drivetrain, 
        Shooter shooter, 
        Targeting targeting
    ) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_targeting = targeting;
    }

    @Override
    public void initialize() {
        // Get the robot's current pose
        robotPose = m_drivetrain.getState().Pose;
        
        angleToRotate = m_targeting.angleToTarget(robotPose, m_targeting.getTargetPose());
        
        // Set a reference pose for the robot
        referencePose = robotPose.rotateBy(new Rotation2d(Units.degreesToRadians(angleToRotate)));

        m_drivetrain.rotateToAngle(new Rotation2d(Units.degreesToRadians(angleToRotate)));
    }

    @Override
    public void execute() {
        if (m_drivetrain.getState().Pose.getRotation().getDegrees() - referencePose.getRotation().getDegrees() < 5) { // If the robot is within 5 degrees of the target angle
            double distanceToTarget = m_targeting.distanceFromHub(m_drivetrain.getState().Pose);

            // Use the distance and angle to set the shooter speed and angle
            m_shooter.setFlywheelRPM(m_targeting.getRPMForDistance(distanceToTarget));
            m_shooter.setPitchAngle(m_targeting.getAngleForDistance(distanceToTarget));
        } else {
            // do nothing while driving towards the shooting pose
        }
    }
}
