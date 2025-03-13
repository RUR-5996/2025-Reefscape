package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class SwerveConstants {
    public static final double driveKP = 0.75;
    public static final double driveKI = 0;
    public static final double driveKD = 0;
    public static final double deriveKIzone = 300;

    public static final double steerKP = 0.01;
    public static final double steerKI = 0;
    public static final double steerKD = 0;

    public static final double DRIVE_MOTOR_GEARING = 5.355;
    public static final double STEER_MOTOR_GEARING = 21.43;
    public static final double STEER_MOTOR_COEFFICIENT = 1.0 / STEER_MOTOR_GEARING * 360.0;
    public static final double WHEEL_RADIUS_METERS = 0.10/2;
    public static final double FALCON_RPM = 6379.0;
    public static final double DRIVE_FACTOR = FALCON_RPM / (60.0 * DRIVE_MOTOR_GEARING) * 2 * Math.PI * WHEEL_RADIUS_METERS; //mps

    public static final double WHEEL_BASE_WIDTH = 0.517;
    public static final double TRACK_WIDTH = 0.516;

    public static final double SECONDSper100MS = .1;
    public static final double TICKSperTALONFX_Rotation = 2048;
    public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING * TICKSperTALONFX_Rotation;
    public static final double METERSperWHEEL_REVOLUTION = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double METERSperROBOT_REVOLUTION = 2 * Math.PI
            * Math.hypot(TRACK_WIDTH, WHEEL_BASE_WIDTH);
    public static final double MAX_SPEED_METERSperSECOND = DRIVE_FACTOR;
    public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
            * (2 * Math.PI);
    public static final double P_ROTATION_CONTROLLER = 0.055;
    public static final double I_ROTATION_CONTROLLER = 0.0;
    public static final double D_ROTATION_CONTROLLER = 0.0;

    public static final Translation2d FL_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
    public static final Translation2d FR_LOC = new Translation2d(SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);
    public static final Translation2d RL_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, SwerveConstants.TRACK_WIDTH / 2);
    public static final Translation2d RR_LOC = new Translation2d(-SwerveConstants.WHEEL_BASE_WIDTH / 2, -SwerveConstants.TRACK_WIDTH / 2);

    public static final InvertedValue FL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
    public static final InvertedValue FR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue RL_DRIVE_INVERT_TYPE = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RR_DRIVE_INVERT_TYPE = InvertedValue.CounterClockwise_Positive;

    public static final boolean FL_STEER_INVERT_TYPE = true;
    public static final boolean FR_STEER_INVERT_TYPE = true;
    public static final boolean RL_STEER_INVERT_TYPE = true;
    public static final boolean RR_STEER_INVERT_TYPE = true;

    public static final double FL_STEER_OFFSET = 0;
    public static final double FR_STEER_OFFSET = 0;
    public static final double RL_STEER_OFFSET = 0;
    public static final double RR_STEER_OFFSET = 0;
  }

  public static class DriverConstants {
    public static final double CONTROLLER_DEBOUNCE_TIME = 0.2;
    public static final double DRIVE_GOVERNOR = 0.6;
    public static final double TURN_GOVERNOR = 0.6;
    public static final double PRECISION_RATIO = 0.2;
  }

  public static final class AutoConstants { //TODO move to SwerveConstants
    public static final PPHolonomicDriveController autoConfig = new PPHolonomicDriveController(new PIDConstants(6.5
    , 0, 0), new PIDConstants(0.5, 0, 0));
  }

  public static class ColourConstants {
    public static double RAINBOW = .75;
    public static double VIOLET = .5;
    public static double PINK = -.37;
    public static double FLASHBANG = -.91;
    public static double BLUEGREEN = -.75;
    public static double LIGHTBLUE = .37;
    public static double DARKBLUE = .75;
    public static double BLUEPINK = .5;
  }

  public static class ElevatorConstants { //all measurements in mm
    public static double DOWN = 310.0; //only updated with current construction!!
    public static double FLOOR0 = 457.2; //TODO update!!
    public static double FLOOR1 = 809.6;
    public static double FLOOR2 = 1209.7;
    public static double FLOOR3 = 1828.8;
  }

  public static class VisionConstants { //all lenghts in m, angles in rad
    public static double reefAprilTagHeight = .308102;
    public static double cameraHeight = 0; //TODO measure
    public static double cameraWidth = 0;
    public static double cameraPitch = 0; //in radians
    public static double cameraFOV = 1.74533; //in radians TODO change
    public static double pictureHeight = 320; //in pixels TODO fact check

    public static double limelightFOVVer = 0.853466; //TODO put to cameraFOV
    public static double limelightFOVHor = 1.090831;
}

public static class IntakeConstants { // angles in rad
    public static double EXTENSION_IN = 0; // TODO add CAD measurements
    public static double EXTENSION_OUT = 60;
    public static double EXTENSION_MID = 10;
    public static double TILT_MOTOR_COEFFICIENT = 18; // 1/20 * 360?
  }
public static class ClimberConstants { // angles in rad
    public static double ANGLE_OUT = 0.349066;
  }

  public static class PathplanningConstants {
  public static Map<Integer, Pose2d> aprilTagPoseMap = Map.ofEntries(//coords in m, angle in deg  NOT positions of tag, but where should robot go
    Map.entry(1, new Pose2d(16.250, 1.000, Rotation2d.fromDegrees(320.0))), //field len 17.55 m
    Map.entry(2, new Pose2d(16.250, 7.000, Rotation2d.fromDegrees(40.0))), //TODO test if values are correct
    Map.entry(3, new Pose2d(11.500, 7.375, Rotation2d.fromDegrees(90.0))),
    Map.entry(4, new Pose2d(9.718, 6.120, Rotation2d.fromDegrees(180.0))),
    Map.entry(5, new Pose2d(9.718, 6.120, Rotation2d.fromDegrees(180.0))),
    Map.entry(6, new Pose2d(12.350, 14.650, Rotation2d.fromDegrees(120.0))),
    Map.entry(7, new Pose2d(11.788, 13.550, Rotation2d.fromDegrees(180.0))),
    Map.entry(8, new Pose2d(12.350, 12.450, Rotation2d.fromDegrees(240.0))),
    Map.entry(9, new Pose2d(13.807, 12.361, Rotation2d.fromDegrees(300.0))),
    Map.entry(10, new Pose2d(14.394, 14.550, Rotation2d.fromDegrees(0.0))),
    Map.entry(11, new Pose2d(13.700, 14.614, Rotation2d.fromDegrees(60.0))),
    Map.entry(12, new Pose2d(1.300, 1.000, Rotation2d.fromDegrees(230.0))),
    Map.entry(13, new Pose2d(1.300, 7.000, Rotation2d.fromDegrees(130.0))),
    Map.entry(14, new Pose2d(7.832, 6.120, Rotation2d.fromDegrees(0.0))),
    Map.entry(15, new Pose2d(7.832, 1.880, Rotation2d.fromDegrees(0.0))),
    Map.entry(16, new Pose2d(6.050, 0.625, Rotation2d.fromDegrees(270.0))),
    Map.entry(17, new Pose2d(3.850, 2.936, Rotation2d.fromDegrees(60.0))),
    Map.entry(18, new Pose2d(3.156, 4.000, Rotation2d.fromDegrees(0.0))),
    Map.entry(19, new Pose2d(3.743, 5.189, Rotation2d.fromDegrees(300.0))),
    Map.entry(20, new Pose2d(5.200, 5.100, Rotation2d.fromDegrees(240.0))),
    Map.entry(21, new Pose2d(5.762, 4.000, Rotation2d.fromDegrees(180.0))),
    Map.entry(22, new Pose2d(5.200, 2.900, Rotation2d.fromDegrees(120.0)))
);
    public static double reefOffset = 0.330; //in m
}

public final class UIConstants {
    public static final class StartPositionUI {
        public static final String POSITION_KEY = "StartPosition";
        public static final String[] POSITIONS = {"Left", "Center", "Right"};
        public static final double SELECTOR_WIDTH = 150;
        public static final double SELECTOR_HEIGHT = 40;
    }

    public static final class CoralPositionUI {
        public static final String POSITION_KEY = "CoralPosition";
        public static final String[] POSITIONS = {"None", "Amp", "Source", "Stage"};
        public static final double SELECTOR_WIDTH = 150;
        public static final double SELECTOR_HEIGHT = 40;
    }
}

}
