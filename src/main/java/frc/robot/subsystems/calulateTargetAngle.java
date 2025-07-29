import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.lang.Math;

  /**
   * Calculates the angle a turret should turn to to aim at a target.
   *  
   * @param robotPose - position and rotation of the robot 
   * @param targetPose - position of the target (rotation is ignored)
   * @return angle the turret should turn to
   */
public static double calculateTargetAngle(Pose2d robotPose, Pose2d targetPose) { // positive angle means counterclockwise
  double robotRotation = robotPose.getRotation().getDegrees();

  double xDiff = robotPose.getX() - targetPose.getX(); // calculates the difference on both axes
  double yDiff = robotPose.getY() - targetPose.getY();

  double resultAngle;
  if (xDiff == 0) { // handle special cases
    if (yDiff > 0) {
      resultAngle = 180;
    } else {
      resultAngle = 0;
    }
  } else if (yDiff == 0) {
    if (xDiff > 0) {
      resultAngle = -90;
    } else {
      resultAngle = 90;
    }
  } else {

    hypotenuse = Math.sqrt(Math.pow(Math.abs(xDiff), 2) + Math.pow(Math.abs(yDiff, 2))); // calculate the angle using the Pythagorean theorem
    resultAngle = Math.sin(Math.abs(yDiff) / hypotenuse);
  }

  if (xDiff < 0) {
    resultAngle = 180 - resultAngle;
  }

  if (yDiff < 0) { // if yDiff is in the negative, flip the angle
    resultAngle *= -1;
  }

  return resultAngle - robotRotation;
}
