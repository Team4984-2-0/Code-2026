package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command that keeps the robot aimed at the current Limelight target while
 * still allowing the driver to control translation.
 */
public class LimelightAutoAim extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdSupplier;
    private final Supplier<Double> ySpdSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final PIDController aimController;
    private final String limelightName;

    public LimelightAutoAim(
            SwerveSubsystem subsystem,
            Supplier<Double> xSpdSupplier,
            Supplier<Double> ySpdSupplier,
            Supplier<Boolean> fieldOrientedSupplier) {
        this(subsystem, xSpdSupplier, ySpdSupplier, fieldOrientedSupplier, VisionConstants.kPrimaryLimelightName);
    }

    public LimelightAutoAim(
            SwerveSubsystem subsystem,
            Supplier<Double> xSpdSupplier,
            Supplier<Double> ySpdSupplier,
            Supplier<Boolean> fieldOrientedSupplier,
            String limelightName) {
        this.swerveSubsystem = subsystem;
        this.xSpdSupplier = xSpdSupplier;
        this.ySpdSupplier = ySpdSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.limelightName = limelightName;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.aimController = new PIDController(
        VisionConstants.kAimKp,
        VisionConstants.kAimKi,
        VisionConstants.kAimKd);
    this.aimController.setTolerance(Units.degreesToRadians(VisionConstants.kAimDeadbandDegrees));
    double integralClamp = Units.degreesToRadians(VisionConstants.kAimIntegralZoneDegrees);
    this.aimController.setIntegratorRange(-integralClamp, integralClamp);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        aimController.reset();
    }

    @Override
    public void execute() {
        double xSpeed = applyDeadband(xSpdSupplier.get());
        double ySpeed = applyDeadband(ySpdSupplier.get());

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        double turningSpeed = 0.0;
        if (LimelightHelpers.getTV(limelightName)) {
            double txRadians = Units.degreesToRadians(LimelightHelpers.getTX(limelightName));
            turningSpeed = aimController.calculate(txRadians, 0.0);
        } else {
            aimController.reset();
        }

    turningSpeed = MathUtil.clamp(turningSpeed,
        -VisionConstants.kAimMaxAngularSpeedRadPerSec,
        VisionConstants.kAimMaxAngularSpeedRadPerSec);

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedSupplier.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > OIConstants.kDeadband ? value : 0.0;
    }
}
