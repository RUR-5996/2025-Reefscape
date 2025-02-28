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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {

    SparkMax intakeMotor;
    RelativeEncoder intakeEncoder;
    SparkClosedLoopController intakeController;

    DoubleSolenoid intakeSolenoid;
    PneumaticsControlModule intakeModule;

    IntakeState intakeState = IntakeState.EMPTY;
    SolenoidState solenoidState = SolenoidState.FORWARD;
    PassiveIntake passiveIntakePrio = PassiveIntake.IN;



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

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);

        SmartDashboard.putBoolean("Passive intake out", false);
    }

    public void prioState() {
        if (SmartDashboard.getBoolean("Passive intake out", false) & passiveIntakePrio == PassiveIntake.IN) {
            passiveIntakePrio = PassiveIntake.OUT;
        }
        else if (SmartDashboard.getBoolean("Passive intake out", true) & passiveIntakePrio == PassiveIntake.OUT){
            passiveIntakePrio = PassiveIntake.IN;
        }
        }

    public Command toggleSolenoid() {
        return Commands.runOnce(() -> {
            if (solenoidState == SolenoidState.FORWARD) {
                intakeSolenoid.set(Value.kReverse);
                solenoidState = SolenoidState.REVERSE;
            } else {
                intakeSolenoid.set(Value.kForward);
                solenoidState = SolenoidState.FORWARD;
            }
        });
    }

    public Command intake() {
        return Commands.run(() -> {
            intakeMotor.set(.5);
        });
    }

    public String getIntakeState() {
        return intakeState.toString();
    }

    public String getSolenoidState() {
        return solenoidState.toString();
    }

    private enum PassiveIntake {
        OUT,
        IN,
        ERROR,
    }

    private enum IntakeState {
        EMPTY,
        FULL,
        ERROR,
    }

    private enum SolenoidState {
        FORWARD,
        REVERSE,
        ERROR,
    }
}
