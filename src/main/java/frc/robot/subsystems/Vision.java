package frc.robot.subsystems;

import com.fasterxml.jackson.core.util.ReadConstrainedTextBuffer;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

import java.util.concurrent.locks.Condition;

import javax.xml.crypto.dsig.TransformException;
import javax.xml.transform.Result;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

    private static Vision VISION;
    static SwerveDrive SWERVE;

    VisionState state = VisionState.APRIL;
    PhotonTrackedTarget tag;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform2d cameraToRobot;

    public Vision() {
        tag = new PhotonTrackedTarget();
        SWERVE = SwerveDrive.getInstance();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        cameraToRobot = new Transform2d(null, null, new Rotation2d(0)); //rotation is in rad
    }

    PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    public static Vision getInstance() {
        if(VISION == null) {
            VISION = new Vision();
        }
        return VISION;
    }

    public void report() {
        SmartDashboard.putString("Pipeline", getVisionState());
        SmartDashboard.putNumber("TagID", (tag.getFiducialId())); //reports apriltag
    }

    /*public PhotonTrackedTarget getTarget() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }
        return null;
    }*/

    public Command april() {
        return Commands.runOnce(() -> {
            state = VisionState.APRIL;
            camera.setPipelineIndex(0); //set pipeline to apriltag
            var result = camera.getLatestResult();
            if (result.hasTargets()) { 
                tag = result.getBestTarget();
    }});
    }

    public Command object() {
        return Commands.runOnce(() -> {
            state = VisionState.OBJECT;
            camera.setPipelineIndex(1); //set pipeline to object 
            var result = camera.getLatestResult();
            if (result.hasTargets()) { 
                tag = result.getBestTarget();
            }
        });
    }


    public String getVisionState() {
        return state.toString();
    }

    private enum VisionState {
        APRIL,
        OBJECT,
    }

    public Command getPos() {
        return Commands.runOnce(() -> {
            var result = camera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            Pose2d targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
            Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
            Constants.VisionConstants.cameraHeight, Constants.VisionConstants.reefAprilTagHeight, Constants.VisionConstants.cameraPitch, Math.toRadians(target.getPitch()), Rotation2d.fromDegrees(-target.getYaw()), SWERVE.gyro.getRotation2d(), targetPose, cameraToRobot);
        }); //TODO accurate camera height, camera offset
    }
}
