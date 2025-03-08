package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {

    private static Elevator ELEVATOR;

    ElevatorState state = ElevatorState.DOWN;
    public ElevatorState manual = ElevatorState.DOWN;


    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;
    public SparkMax leftMotor;
    public SparkMax rightMotor;

    PIDController backupController;

    SparkClosedLoopController leftController;
    SparkClosedLoopController rightController;

    public Elevator() {
        leftMotor = new SparkMax(5, MotorType.kBrushless);
        rightMotor = new SparkMax(6, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(10)
            .i(0)
            .d(0)
            .outputRange(-0.3, 0.3)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180, 180);
        config.inverted(false);
        leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPosition(0);
        leftController = leftMotor.getClosedLoopController();

        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPosition(0);
        rightController = rightMotor.getClosedLoopController();

        SmartDashboard.putBoolean("algaePrio", true);

        // create buttons
        SmartDashboard.putBoolean("DOWN", true);
        SmartDashboard.putBoolean("FLOOR0", false);
        SmartDashboard.putBoolean("FLOOR1", false);
        SmartDashboard.putBoolean("FLOOR2", false);
        SmartDashboard.putBoolean("FLOOR3", false);
    }

    public static Elevator getInstance() {
        if (ELEVATOR == null) {
            ELEVATOR = new Elevator();
        }
        return ELEVATOR;
    }


    public Command checkElevator(ElevatorState target, Intake left, Intake right) { // takes intake instance
        return Commands.either(
                Commands.parallel(
                        Commands.either(left.intakeMid().andThen(elevate(target)), Commands.none(), () -> (left.intakePosition == Intake.IntakePosition.IN)),
                        Commands.either(right.intakeMid().andThen(elevate(target)), Commands.none(), () -> (right.intakePosition == Intake.IntakePosition.IN)))
                .andThen(elevate(target)),
                elevate(target),
                () -> (state == ElevatorState.DOWN || state == ElevatorState.FLOOR0)
        );
    }

    private Command elevate(ElevatorState floor) { //TODO smazat void a bool
        return Commands.runOnce(() -> {
            //double rotations = getMotorRotations((floorToMm(floor)-frc.robot.Constants.ElevatorConstants.DOWN));
            double rotations = getStateRotations(floor);
            leftController.setReference(rotations, SparkMax.ControlType.kPosition);
            rightController.setReference(rotations, SparkMax.ControlType.kPosition);
            state = floor;
            SmartDashboard.putNumber("rotations", rotations);
        });
    }


    public void elevate(ElevatorState floor, boolean nic) { //takes target floor
        //double rotations = getStateRotations(floor);
        double rotations = (getMotorRotations(floorToDownMM(floor))); //TODO fix mezifloor travel
        SmartDashboard.putNumber("Target rotations", getStateRotations(floor));
        SmartDashboard.putNumber("Current rotations", getStateRotations(state));
        SmartDashboard.putNumber("rotations", rotations);
        leftController.setReference(rotations, SparkMax.ControlType.kPosition);
        rightController.setReference(rotations, SparkMax.ControlType.kPosition);
        state = floor;
    }

    public void report() {
        SmartDashboard.putString("floor", state.toString());
        SmartDashboard.putNumber("left motor", leftEncoder.getPosition());
        SmartDashboard.putNumber("right motor", rightEncoder.getPosition());
    }

    public void checkManual() {
        if (SmartDashboard.getBoolean(ElevatorState.DOWN.toString(), false) && !(ElevatorState.DOWN == manual)) {
            SmartDashboard.putBoolean(manual.toString(), false);
            manual = ElevatorState.DOWN;
        } else if (SmartDashboard.getBoolean(ElevatorState.FLOOR0.toString(), false) && !(ElevatorState.FLOOR0 == manual)) {
            SmartDashboard.putBoolean(manual.toString(), false);
            manual = ElevatorState.FLOOR0;
        } else if (SmartDashboard.getBoolean(ElevatorState.FLOOR1.toString(), false) && !(ElevatorState.FLOOR1 == manual)) {
            SmartDashboard.putBoolean(manual.toString(), false);
            manual = ElevatorState.FLOOR1;
        } else if (SmartDashboard.getBoolean(ElevatorState.FLOOR2.toString(), false) && !(ElevatorState.FLOOR2 == manual)) {
            SmartDashboard.putBoolean(manual.toString(), false);
            manual = ElevatorState.FLOOR2;
        } else if (SmartDashboard.getBoolean(ElevatorState.FLOOR3.toString(), false) && !(ElevatorState.FLOOR3 == manual)) {
            SmartDashboard.putBoolean(manual.toString(), false);
            manual = ElevatorState.FLOOR3;
        }
    }

    AlgaePrioState algaePrioState = AlgaePrioState.ON;

    public void AlgaePrioUpdate() {
        if (SmartDashboard.getBoolean("algaePrio", true)) {
            algaePrioState = AlgaePrioState.ON;
        } else {
            algaePrioState = AlgaePrioState.OFF;
        }
        SmartDashboard.putString("algae prio", getAlgaePrio());
    }


    public String getElevatorState() {
        return state.toString();
    }
    public String getAlgaePrio() {
        return algaePrioState.toString();
    }
    public String getManual(){
        return manual.toString();
    }

    public void setHeight() {
        leftController.setReference(getMotorRotations(500), ControlType.kPosition);
    }

    public enum ElevatorState {
        DOWN,
        FLOOR0,
        FLOOR1,
        FLOOR2,
        FLOOR3,
    }

    public enum AlgaePrioState {
        ON,
        OFF,
    }


    private double floorToMm (ElevatorState floor) {//TODO add conversion to DOWN state
        if (floor == ElevatorState.DOWN) {
            return frc.robot.Constants.ElevatorConstants.DOWN;
        } if (floor == ElevatorState.FLOOR0) {
            return frc.robot.Constants.ElevatorConstants.FLOOR0;
        } if (floor == ElevatorState.FLOOR1) {
            return frc.robot.Constants.ElevatorConstants.FLOOR1;
        } if (floor == ElevatorState.FLOOR2) {
            return frc.robot.Constants.ElevatorConstants.FLOOR2;
        } if (floor == ElevatorState.FLOOR3) {
            return frc.robot.Constants.ElevatorConstants.FLOOR3;
        } return 0; //invalid floor inputed
    }

    private double floorToDownMM (ElevatorState floor) {// returns number of mms between wanted floor and down
        if (floor == ElevatorState.FLOOR0) {
            return frc.robot.Constants.ElevatorConstants.FLOOR0 - Constants.ElevatorConstants.DOWN;
        } if (floor == ElevatorState.FLOOR1) {
            return frc.robot.Constants.ElevatorConstants.FLOOR1 - Constants.ElevatorConstants.DOWN;
        } if (floor == ElevatorState.FLOOR2) {
            return frc.robot.Constants.ElevatorConstants.FLOOR2 - Constants.ElevatorConstants.DOWN;
        } if (floor == ElevatorState.FLOOR3) {
            return frc.robot.Constants.ElevatorConstants.FLOOR3 - Constants.ElevatorConstants.DOWN;
        } return 0; //wanted down or invalid floor inputed
    }

    private static double getMotorRotations(double height_requested_m) { //enter double of elevator extension in m, returns number of windings of the motor required to achieve that

        double thickness_in_mm = 0.0025;
        double inner_diam_in_m = 0.024;
        double max_height_in_m = 1.6;
        double max_windings = 12;
        double requested_height_fraction = height_requested_m/max_height_in_m;
        if (requested_height_fraction > 1) {
            requested_height_fraction = 1;
        }

        double requested_motor_rotation = Math.abs((thickness_in_mm - inner_diam_in_m + Math.sqrt((Math.pow(inner_diam_in_m - thickness_in_mm, 2) + ((4*thickness_in_mm*height_requested_m) / (Math.PI))))) / (2*thickness_in_mm));

        if (requested_motor_rotation > max_windings) {
            requested_motor_rotation = max_windings;
        }

        return requested_motor_rotation * 5; //5 kvuli prevodovce
    }

    double getStateRotations(ElevatorState state) {
        switch (state) {
            case DOWN:
                return 0;
            case FLOOR0:
                return 5;
            case FLOOR1:
                return 10;
            case FLOOR2:
                return 40;
            case FLOOR3:
                return 54;
            default:
                return 0;
        }
    }

    public Command addToDesiredState() {
        return Commands.runOnce(() -> {
            switch (manual) {
                case DOWN:
                    manual = ElevatorState.FLOOR0;
                case FLOOR0:
                    manual = ElevatorState.FLOOR1;
                case FLOOR1:
                    manual = ElevatorState.FLOOR2;
                case FLOOR2:
                    manual = ElevatorState.FLOOR3;
                case FLOOR3:
                    manual = ElevatorState.FLOOR3;
            }
        });
    }

    public Command subtractFromDesiredState() {
        return Commands.runOnce(() -> {
            switch (manual) {
                case DOWN:
                    manual = ElevatorState.DOWN;
                case FLOOR0:
                    manual = ElevatorState.DOWN;
                case FLOOR1:
                    manual = ElevatorState.FLOOR0;
                case FLOOR2:
                    manual = ElevatorState.FLOOR1;
                case FLOOR3:
                    manual = ElevatorState.FLOOR2;
            }
        });
    }
}
