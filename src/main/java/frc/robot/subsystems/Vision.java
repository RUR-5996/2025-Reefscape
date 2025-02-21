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

import java.lang.annotation.Target;
import java.util.List;
import java.util.concurrent.locks.Condition;

import javax.xml.crypto.dsig.TransformException;
import javax.xml.transform.Result;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

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
        cameraToRobot = new Transform2d(0, 0, new Rotation2d(0)); //rotation is in rad
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
        april(true);
        //SmartDashboard.putNumber("TagID", (tag.getFiducialId())); //reports apriltag
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

    public void april(boolean nic){
        state = VisionState.APRIL;
        camera.setPipelineIndex(0); //set pipeline to apriltag
        var result = camera.getLatestResult();
        if (result.hasTargets()) { 
            tag = result.getBestTarget();
            SmartDashboard.putNumber("TagID", (tag.getFiducialId()));
        }
    }
    public Command object() {
        return Commands.runOnce(() -> {
            state = VisionState.OBJECT;
            camera.setPipelineIndex(1); //set pipeline to object 
            var result = camera.getLatestResult();
            if (result.hasTargets()) { 
                tag = result.getBestTarget();
                List<TargetCorner> corners = tag.getMinAreaRectCorners();
                TargetCorner corner = corners.get(0);
                SmartDashboard.putNumber("corner 0 x", corner.x);
                }
            }
        );
    }

    public void object(boolean nic) {
        state = VisionState.OBJECT;
        camera.setPipelineIndex(1); //set pipeline to object
        var result = camera.getLatestResult();
        if (result.hasTargets()) { 
            tag = result.getBestTarget();
            List<TargetCorner> corners = tag.getMinAreaRectCorners(); 
            // 0:bottom left, 1:bottom right, 2:top right, 3:top left
            SmartDashboard.putNumber("x corner 0", corners.get(0).x);
            SmartDashboard.putNumber("y corner 0", corners.get(0).y);
            SmartDashboard.putNumber("x corner 1", corners.get(1).x);
            SmartDashboard.putNumber("y corner 1", corners.get(1).y);
            SmartDashboard.putNumber("x corner 2", corners.get(2).x);
            SmartDashboard.putNumber("y corner 2", corners.get(2).y);
            SmartDashboard.putNumber("x corner 3", corners.get(3).x);
            SmartDashboard.putNumber("y corner 3", corners.get(3).y);
            } 
    }

    public static PhotonTrackedTarget simulateForLoop(int i, List<PhotonTrackedTarget> list) {
        if (i+1 == list.size()) {
            return list.get(i);
        }
        return simulateForLoop(i + 1, list);
    }

    public void coralCheck(double distance, PhotonPipelineResult result) { //takes distance from coral in m
        // get conversion factor& other math things
        double x = Math.sqrt(Math.pow(distance, 2) + Math.pow(Constants.VisionConstants.cameraHeight, 2));
        double height_m = Math.tan(Constants.VisionConstants.cameraFOV) * x;
        double conversion_factor = height_m/Constants.VisionConstants.pictureHeight; //multiply pixels by this= get irl
        double cam_height_px = Math.tan(Constants.VisionConstants.cameraPitch) * distance;
        double floor0_px = (cam_height_px - Constants.VisionConstants.cameraHeight / conversion_factor) + 0.45 /conversion_factor;
        //TODO get bounds for positions on reef
        //TODO detect objects
        List<PhotonTrackedTarget> targets = result.getTargets();

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
