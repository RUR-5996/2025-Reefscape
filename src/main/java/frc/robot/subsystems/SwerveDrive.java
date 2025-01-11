package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase{

    public DriveTrain DRIVETRAIN;
    private static SwerveDrive SWERVE;

    double xSpeed = 0;
    double ySpeed = 0;
    double rotation = 0;
    double holdAngle = 0;
    double defaultHoldAngle = 0;
    double defaultAngle = 0;
    double rotationControllerOutput;

    boolean fieldRelative = true; 
    boolean assistedDrive = false;
    boolean rampToggle = false;
    boolean slowmode = false;
    boolean aprilTagDetected = false;
    boolean holdAngleEnabled = false;
    boolean tagControllerEnabled = false;
    boolean noteControllerEnabled = false;

    Pose2d robotPose = new Pose2d();
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    ChassisSpeeds chassisSpeeds;
    SwerveDrivePoseEstimator m_odometry;
    static AHRS gyro;

    PIDController angleHoldController = new PIDController(10, 0, 0); // edit the vals
    PIDController rotationController;

    public SwerveDrive() {

        DRIVETRAIN = DriveTrain.getInstance();

        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
        gyro.reset();
        gyro.setAngleAdjustment(0);

        m_odometry = new SwerveDrivePoseEstimator(
            DRIVETRAIN.swerveKinematics,
            getHeading(),
            DRIVETRAIN.getModulePositions(),
            robotPose);

        setFieldOriented();

        angleHoldController.disableContinuousInput();
        angleHoldController.setTolerance(Math.toRadians(2)); // the usual drift

        rotationController = new PIDController(
            SwerveConstants.P_ROTATION_CONTROLLER, 
            SwerveConstants.I_ROTATION_CONTROLLER, 
            SwerveConstants.D_ROTATION_CONTROLLER);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(2);

        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d(Math.toRadians(gyro.getAngle())));
    }

    /**
     * Function with the decision tree for driving the chassis
     */
    @Override
    public void periodic() {
        m_odometry.update(getHeading(), DRIVETRAIN.getModulePositions());

        robotPose = updateOdometry();

    }

    public static SwerveDrive getInstance() {
        if(SWERVE == null) {
            SWERVE = new SwerveDrive();
        }
        return SWERVE;
    }
    
    public Command toggleSlowMode() { //TODO deprecate
        return Commands.runOnce(() -> {slowmode = !slowmode;});
    }
    
    public void setSlowmodeFlag(boolean flag) {
        slowmode = flag;
    }

    public double updateRotationController() {
        rotationControllerOutput = rotationController.calculate(
            getOdometryDegrees(),
            holdAngle);
        rotationControllerOutput = MathUtil.clamp(rotationControllerOutput, -1, 1);
        return -rotationControllerOutput * SwerveConstants.MAX_SPEED_RADIANSperSECOND;
    }

    public Pose2d updateOdometry() {
        return m_odometry.update(getHeading(), DRIVETRAIN.getModulePositions());
    }

    public void resetOdometry(Pose2d newPose) {
        m_odometry.resetPosition(newPose.getRotation(), DRIVETRAIN.getModulePositions(), newPose);
    }

    public boolean getAtGoal() {
        return rotationController.atSetpoint();
    }

    public boolean getSlowMode() {
        return slowmode;
    }

    public double getOdometryDegrees() {
        return getPose().getRotation().getDegrees();
    }

    public double getGyroDegrees() {
        return getHeading().getDegrees();
    }  

    public DoubleSupplier supplyOdometryDegrees() {
        DoubleSupplier angle = () -> getOdometryDegrees();
        return angle;
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> gyro.reset());
    }

    public double getyMeters() {
        return m_odometry.getEstimatedPosition().getY();
    }

    public double getxMeters() {
        return m_odometry.getEstimatedPosition().getX();
    }

    public double getHoldAngle() {
        return holdAngle;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public Pose2d getOdometryPose() {
        return m_odometry.getEstimatedPosition();
    }

    public Pose2d getPose() {
        return robotPose;
    }
    
    public static Rotation2d getHeading() {
        return new Rotation2d(Math.toRadians(gyro.getAngle()));
    }

    public SwerveDriveKinematics getKinematics() {
        return DRIVETRAIN.swerveKinematics;
    }

    public boolean getHoldAngleEnabled() {
        return holdAngleEnabled;
    }

    public ChassisSpeeds getActualSpeeds() {
        return DRIVETRAIN.getSpeeds();
    }

    public void setFieldOriented() {
        fieldRelative = true;
        holdAngle = Math.toRadians(gyro.getAngle());
    }

    public void setHoldAngle(double angle) { //ve STUPNICH
        holdAngle = angle;
    }

    public void setHoldAngleFlag(boolean flag) {
        holdAngleEnabled = flag;
    }

    public void setOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose.getRotation(), modulePositions, pose);
    }

    public void setAutoModuleStates(SwerveModuleState[] states) {
        DRIVETRAIN.setModuleSpeeds(states);
    }

    public void setAutoChassisSpeeds(ChassisSpeeds speeds) {
        chassisSpeeds = speeds;
        setAutoModuleStates(getKinematics().toSwerveModuleStates(speeds));
    }

    public Command joystickDrive(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, SwerveDrive drive) {
        return Commands.run(() -> {
            SwerveModuleState[] states;

            double leftX = lx.getAsDouble();
            double leftY = ly.getAsDouble();
            double rightX = rx.getAsDouble();

            xSpeed = deadzone(leftX) * deadzone(leftX) * Math.signum(leftX) * SwerveConstants.MAX_SPEED_METERSperSECOND * DriverConstants.DRIVE_GOVERNOR;
            ySpeed = deadzone(leftY) * deadzone(leftY) * Math.signum(leftY) * SwerveConstants.MAX_SPEED_METERSperSECOND * DriverConstants.DRIVE_GOVERNOR;
            rotation = deadzone(rightX) * deadzone(rightX) * Math.signum(rightX) * SwerveConstants.MAX_SPEED_RADIANSperSECOND * DriverConstants.TURN_GOVERNOR;


            if(slowmode) { 
                xSpeed = xSpeed * DriverConstants.PRECISION_RATIO;
                ySpeed = ySpeed * DriverConstants.PRECISION_RATIO;
                rotation = rotation * DriverConstants.PRECISION_RATIO;
            }

            if(holdAngleEnabled) {
                rotation = updateRotationController();
            }
            else {
                //rotation = deadzone(rightX) * SwerveConstants.MAX_SPEED_RADIANSperSECOND * DriverConstants.TURN_GOVERNOR;
            }
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, new Rotation2d(Math.toRadians(gyro.getAngle())));
            states = DRIVETRAIN.swerveKinematics.toSwerveModuleStates(chassisSpeeds);


            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED_METERSperSECOND);

            DRIVETRAIN.setModuleSpeeds(states);
        }, drive);
    }

    public double deadzone(double input) {
        if (Math.abs(input) < 0.2) {
            return 0;
        } else {
            return input;
        }
    }

    public boolean isRed() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    void gyroReset(boolean input) {
        if(input) {
            m_odometry.resetPosition(getHeading(), DRIVETRAIN.getModulePositions(), new Pose2d(robotPose.getTranslation(), new Rotation2d(0)));
        }
    }
}