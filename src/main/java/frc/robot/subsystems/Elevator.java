package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class Elevator extends SubsystemBase {

    ElevatorState state = ElevatorState.DOWN;

    public RelativeEncoder liftEncoder;
    public SparkMax liftMotor;

    PIDController backupController;

    SparkClosedLoopController liftController;

    public void init() {            

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.02)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180, 180);
        liftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        liftEncoder = liftMotor.getEncoder(); 
        liftController = liftMotor.getClosedLoopController();
        }
    public ElevatorState elevate(ElevatorState floor) {
        if (state == floor) { //if already on correct floor, return
            return floor;
        }
        //TODO elevator
        return state;
    }

    public String getElevatorState() {
        return state.toString();
    }
    public void setHeight() {
        liftController.setReference(getElevatorPosition(500), ControlType.kPosition);
    }

    private enum ElevatorState {
        DOWN,
        FLOOR0,
        FLOOR1,
        FLOOR2,
        FLOOR3,
    }

    public double getElevatorPosition(double height_requested_mm) { //enter double of elevator extension in mm, returns number of windings of the motor required to achieve that
        double thickness_in_mm_t = 2.5;
        double max_height = 1600;
        double requested_height_fraction = height_requested_mm/max_height;
        double max_windings = 12;
        double inner_diam_in_mm_d = 24;
        if (requested_height_fraction > 1) {
            requested_height_fraction = 1;
        }
        double requested_motor_rotation = ((-inner_diam_in_mm_d*Math.PI + Math.sqrt(Math.pow(inner_diam_in_mm_d,2)*Math.pow(Math.PI,2)
         + 4+Math.PI+Math.pow(thickness_in_mm_t,2)))/2*Math.PI*thickness_in_mm_t);

        if (requested_motor_rotation > max_windings) {
            requested_motor_rotation = max_windings;
        }


        return requested_motor_rotation * 5; //*5 kvuli prevodovce */
    }
}
