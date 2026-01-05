package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

public class VisionTelemetry extends SubsystemBase {

    Vision visionSubsystem;

    public VisionTelemetry(Vision visionSubsystem) {
        this.visionSubsystem = visionSubsystem;

    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> estimatedGlobalpose = visionSubsystem.getEstimatedGlobalPose(Cameras.LEFT_CAM);

        if (estimatedGlobalpose.isPresent()) {
            SmartDashboard.putNumberArray("EstimatedGlobalPose",
                    estimatedGlobalpose.get().estimatedPose.toMatrix().getData());
        } else {
            SmartDashboard.putNumberArray("EstimatedGlobalPose", new double[0]);
        }

    }

}
