package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SwerveDrive;

public class Report {
    Field2d field;

    static SwerveDrive SWERVE;
    PowerDistribution PDP;
    static DriveTrain DRIVETRAIN;
    static Report REPORT;

    public Report() {
        SWERVE = SwerveDrive.getInstance();
        DRIVETRAIN = DriveTrain.getInstance();
        PDP = new PowerDistribution(0, ModuleType.kCTRE);

        setupField();
    }

    public void setupField() {
        field = new Field2d();
        SmartDashboard.putData("field", field);
        field.setRobotPose(SWERVE.getOdometryPose());
        //reportSwerve();
    }

    public void periodic() {
        field.setRobotPose(SWERVE.getOdometryPose());
    }

    public static Report getInstance() {
        if(REPORT == null) {
            REPORT = new Report();
        }
        return REPORT;
    }

    public static void reportSwerve() {
        SmartDashboard.putData("Swerve Drive", new Sendable() { //all headings are in degrees
            @Override
                public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> DRIVETRAIN.flModule.getSteerAngle(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> DRIVETRAIN.flModule.getSpeed(), null);

                builder.addDoubleProperty("Front Right Angle", () -> DRIVETRAIN.frModule.getSteerAngle(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> DRIVETRAIN.frModule.getSpeed(), null);

                builder.addDoubleProperty("Back Left Angle", () -> DRIVETRAIN.rlModule.getSteerAngle(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> DRIVETRAIN.rlModule.getSpeed(), null);

                builder.addDoubleProperty("Back Right Angle", () -> DRIVETRAIN.rrModule.getSteerAngle(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> DRIVETRAIN.rrModule.getSpeed(), null);

                builder.addDoubleProperty("Robot Angle", () -> SWERVE.getOdometryDegrees(), null);
            }
        });
    }
}
