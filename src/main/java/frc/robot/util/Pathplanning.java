package frc.robot.util;
import java.lang.Math;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class Pathplanning {

    public static Pose2d tagIDToPose(Integer TagID) {
        return Constants.PathplanningConstants.aprilTagPoseMap.get(TagID);
    }

    public static PathPlannerPath getPath(Pose2d start, Integer tagID) {
        return getPath(start, Constants.PathplanningConstants.aprilTagPoseMap.get(tagID));
    }

    public static PathPlannerPath getPath(Pose2d start, Integer tagID, String offsetDirection) { //offsetDirection can be to the "left" or to the "right"
        Pose2d end = Constants.PathplanningConstants.aprilTagPoseMap.get(tagID);
        double x = end.getX();
        double y = end.getY();
        double angle = end.getRotation().getDegrees() % 360;
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
        double a = Math.sin(alpha) * offset; //opposite
        double b = Math.cos(alpha) * offset; //adjacent

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

    public static double getLeftTagID (double tagID) { //returns ID of a tag on the reef left of input tag
        if (tagID < 22 && tagID >= 17) {
            return tagID - 1;
        } else if (tagID == 22) {
            return 17;
        } else if (tagID <= 11 && tagID > 6) {
            return tagID - 1;
        } else if (tagID == 6) {
            return 11;
        } else {
            return 0;
        }
    }

    public static double getRightTagID (double tagID) { //returns ID of a tag on the reef right of input tag
        if (tagID <= 22 && tagID > 17) {
            return tagID + 1;
        } else if (tagID == 17) {
            return 22;
        } else if (tagID < 11 && tagID >= 6) {
            return tagID + 1;
        } else if (tagID == 11) {
            return 6;
        } else {
            return 0;
        }
    }

    public static Command getPathCommand(Pose2d start, Integer tagID) {
        return AutoBuilder.followPath(getPath(start, tagID));
    }

    public static Command getPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    public static Pose2d StartPoseLeft = new Pose2d(7, 5.850, Rotation2d.fromDegrees(0));
    public static Pose2d StartPoseRight = new Pose2d(7, 2.200, Rotation2d.fromDegrees(0));
    public static Pose2d FirstDeployLeft = new Pose2d(3.85, 5.15, Rotation2d.fromDegrees(-60));
    public static Pose2d FirstDeployRight = new Pose2d(3.53, 3.13, Rotation2d.fromDegrees(60));
    public static Pose2d SecondDeployLeft = new Pose2d(4.2, 5.35, Rotation2d.fromDegrees(-60));
    public static Pose2d SecondDeployRight = new Pose2d(3.8, 3, Rotation2d.fromDegrees(60));
    public static Pose2d ThirdDeployLeft = new Pose2d(5.2, 5.0, Rotation2d.fromDegrees(-120));
    public static Pose2d ThirdDeployRight = new Pose2d(4.8, 2.7, Rotation2d.fromDegrees(120));
    public static Pose2d FourthDeployLeft = new Pose2d(5.5, 4.9, Rotation2d.fromDegrees(-120));
    public static Pose2d FourthDeployRight = new Pose2d(5, 2.9, Rotation2d.fromDegrees(120));
    public static Pose2d LoadingLeft = new Pose2d(1.4, 7.3, Rotation2d.fromDegrees(120));
    public static Pose2d LoadingRight = new Pose2d(1.4, 0.8, Rotation2d.fromDegrees(-120));

    public static void loadAutoPath(boolean startingRight) {
        List<Waypoint> waypoints;
        if (true) {
            waypoints = PathPlannerPath.waypointsFromPoses(
                StartPoseRight,
                FirstDeployRight,
                LoadingRight,
                SecondDeployRight,
                LoadingRight,
                ThirdDeployRight,
                LoadingRight,
                FourthDeployRight
            );
        }
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        return;
    }

    public static Command getReefCommand(Integer startTagID, String direction) {
        Integer endTagID;
        if (direction.toLowerCase() == "left") {
            endTagID = (int)getLeftTagID(startTagID);
        } else {
            endTagID = (int)getRightTagID(startTagID);
        }
        String autoName = startTagID.toString() + "-" + endTagID.toString();
        return AutoBuilder.buildAuto(autoName);
    }
}
