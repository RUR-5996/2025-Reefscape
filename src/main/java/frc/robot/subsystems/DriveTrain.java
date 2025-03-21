package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDef.SwerveModule;


public class DriveTrain {

    private static DriveTrain DRIVETRAIN;

    SparkMax flSteer;
    SparkClosedLoopController flController;
    TalonFX flDrive;
    public SwerveModule flModule;

    SparkMax frSteer;
    SparkClosedLoopController frController;
    TalonFX frDrive;
    public SwerveModule frModule;

    SparkMax rlSteer;
    SparkClosedLoopController rlController;
    TalonFX rlDrive;
    public SwerveModule rlModule;

    SparkMax rrSteer;
    SparkClosedLoopController rrController;
    TalonFX rrDrive;
    public SwerveModule rrModule;

    public SwerveDriveKinematics swerveKinematics;


    public DriveTrain() {
        flSteer = new SparkMax(1, MotorType.kBrushless);
        flController = flSteer.getClosedLoopController();
        //flDrive = new TalonFX(1, "5996");
        flDrive = new TalonFX(1);
        flModule = new SwerveModule(flSteer, SwerveConstants.FL_STEER_INVERT_TYPE, flDrive, SwerveConstants.FL_DRIVE_INVERT_TYPE);

        frSteer = new SparkMax(2, MotorType.kBrushless);
        flController = frSteer.getClosedLoopController();
        //frDrive = new TalonFX(2, "5996");
        frDrive = new TalonFX(2);
        frModule = new SwerveModule(frSteer, SwerveConstants.FR_STEER_INVERT_TYPE, frDrive, SwerveConstants.FR_DRIVE_INVERT_TYPE);

        rlSteer = new SparkMax(3, MotorType.kBrushless);
        flController = rlSteer.getClosedLoopController();
        //rlDrive = new TalonFX(3, "5996");
        rlDrive = new TalonFX(3);
        rlModule = new SwerveModule(rlSteer, SwerveConstants.RL_STEER_INVERT_TYPE, rlDrive, SwerveConstants.RL_DRIVE_INVERT_TYPE);

        rrSteer = new SparkMax(4, MotorType.kBrushless);
        flController = rrSteer.getClosedLoopController();
        //rrDrive = new TalonFX(4,"5996");
        rrDrive = new TalonFX(4);
        rrModule = new SwerveModule(rrSteer, SwerveConstants.RR_STEER_INVERT_TYPE, rrDrive, SwerveConstants.RR_DRIVE_INVERT_TYPE);

        flModule.moduleInit();
        frModule.moduleInit();
        rlModule.moduleInit();
        rrModule.moduleInit();

        swerveKinematics = new SwerveDriveKinematics(SwerveConstants.FL_LOC, SwerveConstants.FR_LOC, SwerveConstants.RL_LOC, SwerveConstants.RR_LOC);

        setSteerToCoast();
        setDriveToCoast();
    }

    public static DriveTrain getInstance() {
        if(DRIVETRAIN == null) {
            DRIVETRAIN = new DriveTrain();
        }
        return DRIVETRAIN;
    }

    public void setModuleSpeeds(SwerveModuleState[] _swerveModuleSates) {
        flModule.setState(_swerveModuleSates[0]);
        frModule.setState(_swerveModuleSates[1]);
        rlModule.setState(_swerveModuleSates[2]);
        rrModule.setState(_swerveModuleSates[3]);
    }

    public void setSteerToCoast() {
        flModule.setSteerToCoast();
        frModule.setSteerToCoast();
        rlModule.setSteerToCoast();
        rrModule.setSteerToCoast();
    }

    public void setDriveToCoast() {
        flModule.setDriveToCoast();
        frModule.setDriveToCoast();
        rlModule.setDriveToCoast();
        rrModule.setDriveToCoast();
    }

    public void setSteerToBrake() {
        flModule.setSteerToBrake();
        frModule.setSteerToBrake();
        rlModule.setSteerToBrake();
        rrModule.setSteerToBrake();
    }

    public void setDriveToBrake() {
        flModule.setDriveToBrake();
        frModule.setDriveToBrake();
        rlModule.setDriveToBrake();
        rrModule.setDriveToBrake();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            flModule.getModulePosition(),
            frModule.getModulePosition(),
            rlModule.getModulePosition(),
            rrModule.getModulePosition()
        };
    }

    public SequentialCommandGroup flTest() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {flController.setReference(.5, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {flDrive.setPosition(.5);
            })
        );
    }

    public Command frTest() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {frController.setReference(.5, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {frDrive.setPosition(.5);
            })
        );
    }

    public Command rlTest() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {rlController.setReference(.5, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {rlDrive.setPosition(.5);
            })
        );
    }

    public Command rrTest() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {rrController.setReference(.5, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {rrDrive.setPosition(.5);
            })
        );
    }

    public Command flTestStop() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {flController.setReference(0, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {flDrive.setPosition(0);
            })
        );
    }

    public Command frTestStop() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {frController.setReference(0, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {frDrive.setPosition(0);
            })
        );
    }

    public Command rlTestStop() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {rlController.setReference(0, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {rlDrive.setPosition(0);
            })
        );
    }

    public Command rrTestStop() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {rrController.setReference(0, SparkMax.ControlType.kPosition);
            }),
            new WaitCommand(1),
            Commands.runOnce(() -> {rrDrive.setPosition(0);
            })
        );
    }

    public ChassisSpeeds getSpeeds() {
        return swerveKinematics.toChassisSpeeds(new SwerveModuleState[]{flModule.getModuleState(), frModule.getModuleState(), rlModule.getModuleState(), rrModule.getModuleState()});
    }
}
