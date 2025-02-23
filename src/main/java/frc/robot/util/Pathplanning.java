package frc.robot.util;

import java.lang.Math;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;

public class Pathplanning {

    public static PathPlannerPath getPath(Pose2d start, Integer tagID) {
        return getPath(start, Constants.PathplanningConstants.aprilTagPoseMap.get(tagID));
    }
    
    public static PathPlannerPath getPath(Pose2d start, Integer tagID, String offsetDirection) { //offsetDirection can be to the "left" or to the "right"
        Pose2d end = Constants.PathplanningConstants.aprilTagPoseMap.get(tagID);
        double x = end.getX();
        double y = end.getY();
        double angle = end.getRotation().getDegrees();
        double offset = Constants.PathplanningConstants.reefOffset;
        Integer modifier = 1;
        if (offsetDirection.toLowerCase() == "right") {
            modifier = -1; //flips the offset direction
        }

        if (angle == 0) { //if only one axis needs to be changed, change it
            return getPath(start, new Pose2d(x, y + offset * modifier, Rotation2d.fromDegrees(angle)));
        } else if (angle == 90) {
            return getPath(start, new Pose2d(x - offset * modifier, y, Rotation2d.fromDegrees(angle)));
        } else if (angle == 180) {
            return getPath(start, new Pose2d(x, y - offset * modifier, Rotation2d.fromDegrees(angle)));
        } else if (angle == 170) {
            return getPath(start, new Pose2d(x + offset * modifier, y, Rotation2d.fromDegrees(angle)));
        }


                //calculate offset of x, y from total offset (hypotenuse)
        double alpha = angle % 90;
        double a = Math.sin(alpha) * offset; //adjacent
        double b = Math.cos(alpha) * offset; //opposite

        if (angle < 90) {
            return getPath(start, new Pose2d(x - b * modifier, y + a * modifier, Rotation2d.fromDegrees(angle)));
        } else if (angle > 90 && angle < 180) {
            return getPath(start, new Pose2d(x - a * modifier, y - b * modifier, Rotation2d.fromDegrees(angle)));
        } else if (angle > 180 && angle < 270) {
            return getPath(start, new Pose2d(x + b * modifier, y - a * modifier, Rotation2d.fromDegrees(angle)));
        } else {
            return getPath(start, new Pose2d(x + a * modifier, y + b * modifier, Rotation2d.fromDegrees(angle)));
        }
    }

    public static PathPlannerPath getPath(Pose2d start, Pose2d end) {
        PathPlannerPath path = new  PathPlannerPath(PathPlannerPath.waypointsFromPoses(start, end), null, null, new GoalEndState(0, end.getRotation()));
        path.preventFlipping = true;
        return path;
    } 

}