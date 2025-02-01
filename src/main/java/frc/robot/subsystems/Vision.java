package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Vision extends SubsystemBase {

    private static Vision VISION;

    VisionState state = VisionState.APRIL;
    PhotonTrackedTarget tag;

    public Vision() {
        tag = new PhotonTrackedTarget();
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
}
