package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.spark.SparkBase.ControlType;


public class Climber extends SubsystemBase {

    private static Climber CLIMBER;

    public ClimberState state = ClimberState.IDLE;

    SparkMax climbMotor;
    RelativeEncoder climbEncoder;
    SparkClosedLoopController climbController;

    DoubleSolenoid climbSolenoid;
    PneumaticsControlModule climbModule;

    public Climber(PneumaticsControlModule pcm) {
        climbMotor = new SparkMax(55, SparkLowLevel.MotorType.kBrushless);

        climbSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        climbModule = pcm;

        SparkMaxConfig intakeConfig = new SparkMaxConfig(); // TODO fix values
        intakeConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeConfig.closedLoop
                .p(1.0)
                .i(0.0)
                .d(0.0)
                .positionWrappingEnabled(true);

        climbMotor.configure(intakeConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        climbEncoder = climbMotor.getEncoder();
        climbController = climbMotor.getClosedLoopController();
        climbEncoder.setPosition(0);

    }

    public static Climber getInstance(PneumaticsControlModule pcm) {
        if(CLIMBER == null) {
            CLIMBER = new Climber(pcm);
        }

        return CLIMBER;
    }


    public Command out(Command climbPrep) {
        return Commands.sequence(
                climbPrep,
                Commands.parallel(
                        Commands.runOnce(() -> {
                            climbSolenoid.set(DoubleSolenoid.Value.kForward);
                            state = ClimberState.OUT;
                        }),
                        Commands.runOnce(() -> {
                            climbController.setReference(Constants.ClimberConstants.ANGLE_OUT, ControlType.kPosition);
                        })
                )
        );
    }
    public Command climb() {
        return Commands.parallel(
                Commands.runOnce(() -> {
                    climbSolenoid.set(Value.kReverse);
                    state = ClimberState.CLIMB;
                }),
                Commands.runOnce(() -> {
                    climbController.setReference(0, ControlType.kPosition);
                })
        );
    }

    public Command tune() {
        return Commands.run(() -> {
            climbMotor.set(-.05);
        });
    }

    public Command stopTune() {
        return Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancel(tune());
            climbMotor.set(0);
            climbEncoder.setPosition(0);
        });
    }

    public String getClimberState() {
        return state.toString();
    }

    public enum ClimberState {
        IDLE,
        OUT,
        CLIMB,
    }
}
