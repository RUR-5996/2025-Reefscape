package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveDef {

    public static class SwerveModule {
        public RelativeEncoder steerEncoder;
        public SparkClosedLoopController steerController;
        public SparkMax steerMotor;
        public TalonFX driveMotor;
        private TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
        private VelocityVoltage m_velocitySetter = new VelocityVoltage(0);
        double rotationsPerWheelRotation = SwerveConstants.DRIVE_MOTOR_GEARING;
        double metersPerWheelRotation = 2*Math.PI*SwerveConstants.WHEEL_RADIUS_METERS;
        double m_driveRotationsPerMeter = rotationsPerWheelRotation/metersPerWheelRotation;

        boolean steerInverted;
        InvertedValue driveInverted;

        PIDController backupController;

        public SwerveModule(SparkMax sMotor, boolean sInverted, TalonFX dMotor, InvertedValue dInverted) {
            steerMotor = sMotor;
            driveMotor = dMotor;
            steerInverted = sInverted;
            driveInverted = dInverted;
        }

        public void moduleInit() {

            driveMotor.getConfigurator().refresh(driveTalonConfig);
            final Slot0Configs DriveMotorGains = new Slot0Configs();
        
            DriveMotorGains.kP = SwerveConstants.driveKP;
            DriveMotorGains.kI = 0;
            DriveMotorGains.kD = 0;

            driveTalonConfig.Slot0 = DriveMotorGains;

            OpenLoopRampsConfigs openLoopConfig = new OpenLoopRampsConfigs();
            openLoopConfig.TorqueOpenLoopRampPeriod = 0.3;
            openLoopConfig.DutyCycleOpenLoopRampPeriod = 0.3;
            openLoopConfig.VoltageOpenLoopRampPeriod = 0.3;

            ClosedLoopRampsConfigs closedLoopConfig = new ClosedLoopRampsConfigs();
            closedLoopConfig.VoltageClosedLoopRampPeriod = 0.3;
            closedLoopConfig.TorqueClosedLoopRampPeriod = 0.3;
            closedLoopConfig.DutyCycleClosedLoopRampPeriod = 0.3;
            driveTalonConfig.OpenLoopRamps = openLoopConfig;
            driveTalonConfig.ClosedLoopRamps = closedLoopConfig;
            driveTalonConfig.MotorOutput.Inverted = driveInverted;
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            CurrentLimitsConfigs currentLimitsConfig = new CurrentLimitsConfigs();
            currentLimitsConfig.StatorCurrentLimit = 30;
            currentLimitsConfig.SupplyCurrentLimit = 30;
            driveTalonConfig.CurrentLimits = currentLimitsConfig;

            FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
            feedbackConfigs.RotorToSensorRatio = 5.36;
            feedbackConfigs.SensorToMechanismRatio = 2*Math.PI * SwerveConstants.WHEEL_RADIUS_METERS;
            driveTalonConfig.Feedback = feedbackConfigs;

            driveMotor.getConfigurator().apply(driveTalonConfig);

            SparkMaxConfig config = new SparkMaxConfig();
            config
                .inverted(steerInverted)
                .idleMode(IdleMode.kBrake);
            config.encoder.positionConversionFactor(SwerveConstants.STEER_MOTOR_COEFFICIENT);
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(1000)
                .i(0.0001)
                .d(0.0001)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-180, 180);
            steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            steerEncoder = steerMotor.getEncoder(); 
            steerController = steerMotor.getClosedLoopController();

            resetSteerEncoder();

            backupController = new PIDController(SwerveConstants.steerKP, SwerveConstants.steerKI, SwerveConstants.steerKD);
            backupController.setTolerance(1);
        }

        public void setDriveToCoast() { 
            driveMotor.getConfigurator().refresh(driveTalonConfig);
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            driveMotor.getConfigurator().apply(driveTalonConfig);
        }

        public void setSteerToCoast() {
            SparkMaxConfig coast = new SparkMaxConfig();
            coast.idleMode(IdleMode.kCoast);
            steerMotor.configure(coast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void setDriveToBrake() {
            driveMotor.getConfigurator().refresh(driveTalonConfig);
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveMotor.getConfigurator().apply(driveTalonConfig);
        }

        public void setSteerToBrake() {
            SparkMaxConfig coast = new SparkMaxConfig();
            coast.idleMode(IdleMode.kBrake);
            steerMotor.configure(coast, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        public void resetSteerEncoder() {
            steerEncoder.setPosition(0);
        }

        public SwerveModulePosition getModulePosition() {
            return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), new Rotation2d(Math.toRadians(steerEncoder.getPosition())));
        }

        public SwerveModuleState getModuleState() {
            return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(Math.toRadians(steerEncoder.getPosition())));
        }

        public void setAngle(double angleToSet) {
            double encoderPosition = steerMotor.getEncoder().getPosition(); //degrees, can be over 360
            double toFullCircle = Math.IEEEremainder(encoderPosition, 360);
            double newAngle = angleToSet + encoderPosition - toFullCircle;
            
            if (newAngle - encoderPosition > 180) {
                newAngle -= 360;
            } else if (newAngle - encoderPosition < -180) {
                newAngle += 360;
            }

            steerController.setReference(newAngle, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0); // runs PID with adjusted angle over 360 degs
            steerMotor.set(MathUtil.clamp(backupController.calculate(steerEncoder.getPosition(), newAngle), -1.0, 1.0));
        }

        public void setState(SwerveModuleState stateToSet) {
            stateToSet.optimize(new Rotation2d(Math.toRadians(steerEncoder.getPosition())));

            setAngle(stateToSet.angle.getDegrees());

            SmartDashboard.putNumber("speed", stateToSet.speedMetersPerSecond); //TODO check if this maxes out
            driveMotor.setControl(m_velocitySetter.withVelocity(stateToSet.speedMetersPerSecond));
        }

        public double getSteerAngle() {
            return steerEncoder.getPosition();
        }

        public double getSpeed() {
            return driveMotor.getVelocity().getValueAsDouble();
        }

        public void testModule() {
            steerController.setReference(90, ControlType.kPosition);
        }

    }
}