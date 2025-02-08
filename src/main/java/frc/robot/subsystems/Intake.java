package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    SparkMax intakeMotor;
    RelativeEncoder intakeEncoder;
    SparkClosedLoopController intakeController;

    Solenoid intakeSolenoid;
    PneumaticsControlModule intakeModule;

    IntakeState intakeState = IntakeState.EMPTY;
    SolenoidState solenoidState = SolenoidState.FALSE; //might be needed to be set to true

    public Intake() {}; //for testing

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

        intakeModule = new PneumaticsControlModule(0);

        intakeSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, frontPiston);
        intakeSolenoid.set(false); //might be needed to be set to true
    }

    public Command toggleSolenoid() {
        return Commands.runOnce(() -> {
            if (solenoidState == SolenoidState.FALSE) {
                intakeSolenoid.set(true);
                solenoidState = SolenoidState.TRUE;
            } else {
                intakeSolenoid.set(false);
                solenoidState = SolenoidState.FALSE;
            }
        });
    }

    public Command intake() {
        return Commands.run(() -> {
            intakeMotor.set(.5);
        }); //TODO
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
