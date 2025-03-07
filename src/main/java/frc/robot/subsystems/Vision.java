package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Pathplanning;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.List;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase {

    private static Vision VISION;
    static SwerveDrive SWERVE;

    VisionState state = VisionState.APRIL;
    PhotonTrackedTarget tag;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform2d cameraToRobot2d;
    Transform3d cameraToRobot3d;
    PhotonPoseEstimator photonPoseEstimator;
    PhotonCamera camera;

    public Vision() {
        camera = new PhotonCamera("Limelight-3");
        cameraToRobot3d = new Transform3d(new Translation3d(0.0, 0.0, 0), new Rotation3d(0,0,0)); //TODO measure
        tag = new PhotonTrackedTarget();
        SWERVE = SwerveDrive.getInstance();
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        cameraToRobot2d = new Transform2d(0, 0, new Rotation2d(0)); //rotation is in rad
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraToRobot3d);
    }

    public static Vision getInstance() {
        if(VISION == null) {
            VISION = new Vision();
        }
        return VISION;
    }

public Pose3d getRobotPose() {
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot3d);
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

    public double getDistanceToTag(PhotonTrackedTarget target) {
        double distanceToTarget = 0.0;
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot3d);
            Pose3d targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
            distanceToTarget = PhotonUtils.getDistanceToPose(robotPose.toPose2d(), targetPose.toPose2d());
        }
        
        return distanceToTarget;
    }

    public void coralCheck(double distance, PhotonPipelineResult result) { //takes distance from coral in m
        // get conversion factor & other math things
        double x = Math.sqrt(Math.pow(distance, 2) + Math.pow(Constants.VisionConstants.cameraHeight, 2));
        double height_m = Math.tan(Constants.VisionConstants.cameraFOV) * x;
        double conversion_factor = height_m/Constants.VisionConstants.pictureHeight; //multiply pixels by this= get irl
        double cam_height_px = Math.tan(Constants.VisionConstants.cameraPitch) * distance;
        double floor0_px = (cam_height_px - Constants.VisionConstants.cameraHeight / conversion_factor) + 0.45 /conversion_factor;
        //TODO get bounds for positions on reef
        //TODO detect objects
        List<PhotonTrackedTarget> targets = result.getTargets();

    }

    public Command seeAprilAndGo() {
        Pose2d robotPose = getRobotPose().toPose2d();
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        int tagID = target.getFiducialId();
        if ((tagID >= 6 && tagID <= 11) || (tagID >= 17 && tagID <= 22)) { //check if tag on reef
            return Pathplanning.getPathCommand(robotPose, tagID);
        } else {
            return Commands.runOnce(() -> {});  //else return empty command
        }
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
            Constants.VisionConstants.cameraHeight, Constants.VisionConstants.reefAprilTagHeight, Constants.VisionConstants.cameraPitch, Math.toRadians(target.getPitch()), Rotation2d.fromDegrees(-target.getYaw()), SWERVE.gyro.getRotation2d(), targetPose, cameraToRobot2d);
        }); //TODO accurate camera height, camera offset
    }
}
