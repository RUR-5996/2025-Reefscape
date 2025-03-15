package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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

    int grabID;

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

    boolean isGrabOn = false;
    double ref = 0;

    public Intake(int grabId, int tiltId, int frontButtonID, int backButtonID) {
        grabID = grabId;
        grabMotor = new SparkMax(grabId, MotorType.kBrushless);
        tiltMotor = new SparkMax(tiltId, MotorType.kBrushless);

        frontButton = new DigitalInput(frontButtonID);
        backButton = new DigitalInput(backButtonID);

        SparkMaxConfig intakeConfig = new SparkMaxConfig(); // config for grab motor, TODO add for tilt
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(0.5);
        intakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(.02)
            .i(0)
            .d(0)
            .outputRange(-0.15, 0.15)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-180, 180);

        grabMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        intakeConfig.encoder.positionConversionFactor(Constants.IntakeConstants.TILT_MOTOR_COEFFICIENT); // temporary TODO
        tiltMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        tiltEncoder = tiltMotor.getEncoder();
        tiltController = tiltMotor.getClosedLoopController();
        tiltEncoder.setPosition(0);


        SmartDashboard.putBoolean("Passive intake out", false);
    }

    public void permaTilt() {
        tiltController.setReference(ref, ControlType.kPosition);
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
            ref = Constants.IntakeConstants.EXTENSION_MID;
            intakePosition = IntakePosition.MID;
        });
    }

    public Command intakeOut() {
        return Commands.runOnce(()-> {
            ref = Constants.IntakeConstants.EXTENSION_OUT;
            tiltController.setReference(Constants.IntakeConstants.EXTENSION_OUT, SparkMax.ControlType.kPosition);
            intakePosition = IntakePosition.OUT;
        });
    }

    public Command intakeIn() {
        return Commands.runOnce(()-> {
            ref = Constants.IntakeConstants.EXTENSION_IN;
            tiltController.setReference(Constants.IntakeConstants.EXTENSION_IN, SparkMax.ControlType.kPosition);
            intakePosition = IntakePosition.IN;
        });
    }

    public Command grabCoral() {
        return Commands.runOnce(() -> {
            isGrabOn = true;
            //CommandScheduler.getInstance().cancel(stopGrab());
            grabMotor.set(-.9);
            intakeState = IntakeState.FULL;
            /*if (backButton.get()) {
                grabMotor.set(0);
                isGrabOn = false;
            }*/
        });
    }

    public Command stopGrab() {
        return Commands.runOnce(()-> {
            grabMotor.set(0);
            if (frontButton.get()|| !backButton.get()) {
                //CommandScheduler.getInstance().cancel(grabCoral());
                grabMotor.set(0);
                isGrabOn = false;
            }
        });
    }

    public Command releaseCoral() {
        return Commands.runOnce(() -> {
            grabMotor.set(.55);
            intakeState = IntakeState.EMPTY;
        });
    }

    public Command stopRelease() {
        return Commands.runOnce(()-> {
            CommandScheduler.getInstance().cancel(releaseCoral());
            grabMotor.set(0);
        });
    }

    public Command tune() {
        return Commands.run(() -> {
            tiltMotor.set(-.05);
        });
    }

    public Command stopTune() {
        return Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancel(tune());
            tiltMotor.set(0);
            tiltEncoder.setPosition(0);
        });
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    public boolean getGrabState() {
        return isGrabOn;
    }

    public void report() {
        SmartDashboard.putNumber("intake" + grabID, tiltEncoder.getPosition());
        SmartDashboard.putNumber("intakeRef" + grabID, ref);
    }

    public enum IntakePosition {
        IN,
        OUT,
        MID, // not out, but elevator can pass
    }

    private enum IntakeState {
        EMPTY,
        FULL,
    }
}
