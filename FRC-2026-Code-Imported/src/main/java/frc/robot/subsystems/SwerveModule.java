package frc.robot.subsystems;

/**
 * Represents a single swerve module (drive + steer). Provides helpers for
 * encoder conversion, absolute angle syncing, and applying desired states.
 */

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    private int power;

    // what does this look like
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    /**
     * @param driveMotorId            CAN ID for the drive motor
     * @param turningMotorId          CAN ID for the steering motor
     * @param driveMotorReversed      whether the drive inversion should be flipped
     * @param turningMotorReversed    whether the steer inversion should be flipped
     * @param absoluteEncoderId       analog input channel wired to the absolute
     *                                encoder
     * @param absoluteEncoderOffset   mechanical zero offset in radians
     * @param absoluteEncoderReversed set true if encoder direction needs flipping
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(absoluteEncoderId);
        power = 3;
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        driveConfig.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        driveConfig.inverted(driveMotorReversed);

        driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        turnConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        turnConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        turnConfig.inverted(turningMotorReversed);

        turningMotor.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        // SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel(),
        // getAbsoluteEncoderRad());
        resetEncoders();
        // SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " start",
        // getAbsoluteEncoderRad());

    }

    /** Sets which of the predefined power buckets the drive stage should use. */
    public void set_speed(int howfast) {
        power = howfast;
    }

    /** @return drive encoder position in meters. */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return steering encoder position in radians. */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /** @return drive encoder velocity in m/s. */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /** @return turning encoder velocity in rad/s. */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /**
     * Converts the absolute encoder voltage to radians using the stored offset.
     * Keeps the value normalized between +/- pi.
     */
    public double getAbsoluteEncoderRad() {
        if (absoluteEncoder.getChannel() == 0) {
            // System.out.println("Swerve[" + absoluteEncoder.getChannel() + "] voltage " +
            // absoluteEncoder.getVoltage());
            // System.out.println("Swerve[" + absoluteEncoder.getChannel() + "] rio5v " +
            // RobotController.getVoltage5V());
        }
        double angle = absoluteEncoder.getVoltage() / (RobotController.getVoltage5V());

        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderRadtest() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        return angle;
    }

    /** Syncs the relative encoders to the absolute reading. */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        double absoluteAngle = MathUtil.angleModulus(getAbsoluteEncoderRad());
        if (!Double.isFinite(absoluteAngle)) {
            absoluteAngle = 0.0;
        }
        turningEncoder.setPosition(absoluteAngle);
    }

    /** @return current state (speed + angle) used by odometry. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /** @return latest pose snapshot for odometry/pose estimation. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    /**
     * Applies a desired state from either teleop driving or an auto path.
     * Optimizes the rotation to keep steering travel short.
     */
    public void setDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = optimizeState(state);
        switch (power) {
            case 1:
                driveMotor.set((optimizedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond) / 1.3);
                break;
            case 2:
                driveMotor.set((optimizedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond) / 2);
                break;
            case 3:
                driveMotor.set((optimizedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond));
                break;
            default:
                driveMotor.set((optimizedState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond) / 2);
                break;
        }
        /*
         * if (speed) {
         * driveMotor.set((state.speedMetersPerSecond /
         * DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/1.3);
         * }
         * else {
         * driveMotor.set((state.speedMetersPerSecond /
         * DriveConstants.kPhysicalMaxSpeedMetersPerSecond)/2);
         * }
         */
        turningMotor.set(turningPidController.calculate(getTurningPosition(), optimizedState.angle.getRadians()));
        // SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "]
        // state", state.toString());
        // SmartDashboard.putNumber("encoder" + absoluteEncoder.getChannel() + " live",
        // getAbsoluteEncoderRad());
    }

    private SwerveModuleState optimizeState(SwerveModuleState desiredState) {
        double currentAngle = getTurningPosition();
        double targetAngle = MathUtil.angleModulus(desiredState.angle.getRadians());
        double delta = MathUtil.inputModulus(targetAngle - currentAngle, -Math.PI, Math.PI);

        double optimizedAngle = currentAngle + delta;
        double optimizedSpeed = desiredState.speedMetersPerSecond;

        if (Math.abs(delta) > Math.PI / 2.0) {
            optimizedSpeed *= -1.0;
            optimizedAngle = MathUtil.angleModulus(optimizedAngle + Math.PI);
        }

        return new SwerveModuleState(optimizedSpeed, new Rotation2d(optimizedAngle));
    }

    /** Stops both drive and steer motors immediately. */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}