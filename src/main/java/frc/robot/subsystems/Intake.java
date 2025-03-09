package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

    // for grabbing coral
    SparkMax grabMotor;

    // for moving intake
    SparkMax tiltMotor;
    RelativeEncoder tiltEncoder;
    SparkClosedLoopController tiltController;

    IntakeState intakeState = IntakeState.EMPTY;
    IntakePosition intakePosition = IntakePosition.IN;

    DigitalInput frontButton;
    DigitalInput backButton;

    public Intake() {}; //for testing

    public Intake(int grabId, int tiltId, int frontButtonID, int backButtonID) {
        grabMotor = new SparkMax(grabId, MotorType.kBrushless);
        tiltMotor = new SparkMax(tiltId, MotorType.kBrushless);

        frontButton = new DigitalInput(frontButtonID);
        backButton = new DigitalInput(backButtonID);

        SparkMaxConfig intakeConfig = new SparkMaxConfig(); // config for grab motor, TODO add for tilt
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeConfig.closedLoop
            .p(1.0)
            .i(0.0)
            .d(0.0)
            .positionWrappingEnabled(true);

        grabMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        intakeConfig.encoder.positionConversionFactor(Constants.IntakeConstants.TILT_MOTOR_COEFFICIENT); // temporary TODO
        tiltMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        tiltEncoder = tiltMotor.getEncoder();
        tiltController = tiltMotor.getClosedLoopController();
        tiltEncoder.setPosition(0);


        SmartDashboard.putBoolean("Passive intake out", false);
    }

    public SequentialCommandGroup grabCoralSequence() {
        return new SequentialCommandGroup(
            intakeOut(), grabCoral()
        );
    }

    public SequentialCommandGroup releaseCoralSequence() {
        return new SequentialCommandGroup(
           stopGrab(), intakeIn(), releaseCoral(), new WaitCommand(.5), stopRelease(), intakeMid()
        );
    }
    public  Command intakeMid() {
        return Commands.runOnce(()-> {
            tiltController.setReference(Constants.IntakeConstants.EXTENSION_MID, SparkMax.ControlType.kPosition);
            intakePosition = IntakePosition.MID;
        });
    }

    public Command intakeOut() {
        return Commands.runOnce(()-> {
            tiltController.setReference(Constants.IntakeConstants.EXTENSION_OUT, SparkMax.ControlType.kPosition);
            intakePosition = IntakePosition.OUT;
        });
    }

    public Command intakeIn() {
        return Commands.runOnce(()-> {
            tiltController.setReference(Constants.IntakeConstants.EXTENSION_IN, SparkMax.ControlType.kPosition);
            intakePosition = IntakePosition.IN;
        });
    }

    public Command grabCoral() {
        return Commands.run(() -> {
            grabMotor.set(.5);
            intakeState = IntakeState.FULL;
            if (backButton.get() == true) {
                grabMotor.set(0);
            }
        });
    }

    public Command stopGrab() {
        return Commands.runOnce(()-> {
            CommandScheduler.getInstance().cancel(grabCoral());
            grabMotor.set(0);
        });
    }

    public Command releaseCoral() {
        return Commands.run(() -> {
            grabMotor.set(-.25);
            intakeState = IntakeState.EMPTY;
        });
    }

    public Command stopRelease() {
        return Commands.runOnce(()-> {
            CommandScheduler.getInstance().cancel(releaseCoral());
            grabMotor.set(0);
        });
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    public enum IntakePosition {
        IN,
        OUT,
        MID, // not out, but elevator can pass
    }

    private enum IntakeState {
        EMPTY,
        FULL,
        ERROR,
    }


}
