package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    SparkMax intakeMotor;
    RelativeEncoder intakeEncoder;
    SparkClosedLoopController intakeController;

    Solenoid frontSolenoid;
    Solenoid backSolenoid;

    IntakeState intakeState = IntakeState.EMPTY;
    SolenoidState solenoidState = SolenoidState.FALSE; //might be needed to be set to true


    public Intake(int motorId, int frontPiston, int backPiston) {
        intakeMotor = new SparkMax(motorId, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        intakeConfig.closedLoop
            .p(1.0)
            .i(0.0)
            .d(0.0)
            .positionWrappingEnabled(true);
        intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        intakeEncoder = intakeMotor.getEncoder();
        intakeController = intakeMotor.getClosedLoopController();
        intakeEncoder.setPosition(0);

        Solenoid frontSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, frontPiston);
        Solenoid backSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, backPiston);
        frontSolenoid.set(false); //might be needed to be set to true
        backSolenoid.set(false);
    }

    public Command toggleSolenoid() {
        return Commands.runOnce(() -> {
            if (solenoidState == SolenoidState.FALSE) {
                frontSolenoid.set(true);
                backSolenoid.set(true);
            } else {
                frontSolenoid.set(false);
                backSolenoid.set(false);
            }
        });
    }

    public Command intake() {
        return Commands.run(() -> {}); //TODO
    }

    public Command toggleIntake() {
        return Commands.runOnce(() -> {}); //TODO
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    public String getSolenoidState() {
        return solenoidState.toString();
    }

    private enum IntakeState {
        EMPTY,
        FULL,
        ERROR,
    }

    private enum SolenoidState {
        TRUE,
        FALSE,
        ERROR,
    }
}
