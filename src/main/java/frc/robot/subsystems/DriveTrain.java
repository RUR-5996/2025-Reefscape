package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDef.SwerveModule;


public class DriveTrain {

    private static DriveTrain DRIVETRAIN;
    
    SparkMax flSteer;
    TalonFX flDrive;
    public SwerveModule flModule;

    SparkMax frSteer;
    TalonFX frDrive;
    public SwerveModule frModule;

    SparkMax rlSteer;
    TalonFX rlDrive;
    public SwerveModule rlModule;

    SparkMax rrSteer;
    TalonFX rrDrive;
    public SwerveModule rrModule;

    public SwerveDriveKinematics swerveKinematics;


    public DriveTrain() {
        flSteer = new SparkMax(1, MotorType.kBrushless);
        flDrive = new TalonFX(1);
        flModule = new SwerveModule(flSteer, SwerveConstants.FL_STEER_INVERT_TYPE, flDrive, SwerveConstants.FL_DRIVE_INVERT_TYPE);

        frSteer = new SparkMax(2, MotorType.kBrushless);
        frDrive = new TalonFX(2);
        frModule = new SwerveModule(frSteer, SwerveConstants.FR_STEER_INVERT_TYPE, frDrive, SwerveConstants.FR_DRIVE_INVERT_TYPE);

        rlSteer = new SparkMax(3, MotorType.kBrushless);
        rlDrive = new TalonFX(3);
        rlModule = new SwerveModule(rlSteer, SwerveConstants.RL_STEER_INVERT_TYPE, rlDrive, SwerveConstants.RL_DRIVE_INVERT_TYPE);

        rrSteer = new SparkMax(4, MotorType.kBrushless);
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

    public ChassisSpeeds getSpeeds() {
        return swerveKinematics.toChassisSpeeds(new SwerveModuleState[]{flModule.getModuleState(), frModule.getModuleState(), rlModule.getModuleState(), rrModule.getModuleState()});
    }
}