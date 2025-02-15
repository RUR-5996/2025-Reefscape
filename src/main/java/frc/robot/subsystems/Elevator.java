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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.Report;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {

    private static Elevator ELEVATOR;

    ElevatorState state = ElevatorState.DOWN;
    ElevatorState manual = ElevatorState.DOWN;


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
            .p(1.5)
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
        //leftEncoder.setPosition(0);
        leftController = leftMotor.getClosedLoopController();

        rightEncoder = rightMotor.getEncoder(); 
        //rightEncoder.setPosition(0);
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
        if(ELEVATOR == null) {
            ELEVATOR = new Elevator();
        }

        return ELEVATOR;
    }

    public Command elevate(ElevatorState floor, boolean nic) { //TODO smazat void a bool
        return Commands.runOnce(() -> {
            //double rotations = getMotorRotations((floorToMm(floor)-frc.robot.Constants.ElevatorConstants.DOWN));
            double rotations = getStateRotations(floor);
            leftController.setReference(rotations, SparkMax.ControlType.kPosition);
            rightController.setReference(rotations, SparkMax.ControlType.kPosition);
            state = floor;
            SmartDashboard.putNumber("rotations", rotations);
        });
    }
    
    public void elevate(ElevatorState floor) { //takes target floor
        //double rotations = getStateRotations(floor);
        double rotations = (getMotorRotations(floorToDownMM(floor))) - (getMotorRotations(floorToDownMM(state))); //TODO fix mezifloor travel
        leftController.setReference(rotations, SparkMax.ControlType.kPosition);
        rightController.setReference(rotations, SparkMax.ControlType.kPosition);
        state = floor;
        SmartDashboard.putNumber("rotations", rotations);
    }

    public void report() {
        SmartDashboard.putString("floor", state.toString());
    }

    public void checkManual() {
        for (ElevatorState f : ElevatorState.values()){
            if ((SmartDashboard.getBoolean(f.toString(), false)) && (!(f==manual))){
                SmartDashboard.putBoolean(manual.toString(), false);
                manual = f;    
                elevate(f);
            }
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

    private double getMotorRotations(double height_requested_m_L) { //enter double of elevator extension in mm, returns number of windings of the motor required to achieve that
        double thickness_in_mm_h = 0.0025;
        double inner_diam_in_m_D0 = 0.024;
        double max_height = 1.6;
        double max_windings = 12;
        double requested_height_fraction = height_requested_m_L/max_height;
        if (requested_height_fraction > 1) {
            requested_height_fraction = 1;
        }

        double requested_motor_rotation = Math.abs((thickness_in_mm_h - inner_diam_in_m_D0 + Math.sqrt((Math.pow(inner_diam_in_m_D0 - thickness_in_mm_h, 2) + ((4*thickness_in_mm_h*height_requested_m_L) / (Math.PI))))) / (2*thickness_in_mm_h));

        if (requested_motor_rotation > max_windings) {
            requested_motor_rotation = max_windings;
        }

        return requested_motor_rotation * 5; //5 kvuli prevodovce
    }

    double getStateRotations(ElevatorState state) {
        switch (state) {
            case DOWN:
                return -15;
            case FLOOR1:
                return 10.0;
            case FLOOR2:
                return 40;
            case FLOOR3:
                return 54;
            default:
                return 0.0;
        }
    }
}
