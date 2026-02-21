package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Uses the Limelight's AprilTag pose estimate to automatically drive toward a tag
 * while keeping the robot centered and aimed at it.
 */
public class LimelightTagFollow extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final String limelightName;

    private final PIDController forwardController;
    private final PIDController strafeController;
    private final PIDController aimController;

    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;

    public LimelightTagFollow(SwerveSubsystem swerveSubsystem) {
        this(swerveSubsystem, VisionConstants.kPrimaryLimelightName);
    }

    public LimelightTagFollow(SwerveSubsystem swerveSubsystem, String limelightName) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightName = limelightName;

        this.forwardController = new PIDController(
                VisionConstants.kTagFollowXP,
                0.0,
                0.0);
        this.forwardController.setTolerance(VisionConstants.kTagFollowTranslationalToleranceMeters);

        this.strafeController = new PIDController(
                VisionConstants.kTagFollowYP,
                0.0,
                0.0);
        this.strafeController.setTolerance(VisionConstants.kTagFollowTranslationalToleranceMeters);

        this.aimController = new PIDController(
                VisionConstants.kAimKp,
                VisionConstants.kAimKi,
                VisionConstants.kAimKd);
        this.aimController.setTolerance(Units.degreesToRadians(VisionConstants.kAimDeadbandDegrees));
        double integralClamp = Units.degreesToRadians(VisionConstants.kAimIntegralZoneDegrees);
        this.aimController.setIntegratorRange(-integralClamp, integralClamp);

        this.forwardLimiter = new SlewRateLimiter(VisionConstants.kTagFollowMaxLinearAccelerationMetersPerSecondSquared);
        this.strafeLimiter = new SlewRateLimiter(VisionConstants.kTagFollowMaxLinearAccelerationMetersPerSecondSquared);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        forwardController.reset();
        strafeController.reset();
        aimController.reset();
                SmartDashboard.putBoolean("TagFollow/HasTarget", false);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(limelightName)) {
            forwardController.reset();
            strafeController.reset();
            aimController.reset();
            swerveSubsystem.stopModules();
                        SmartDashboard.putBoolean("TagFollow/HasTarget", false);
            return;
        }

        Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
        double forwardMeters = targetPose.getX();
        double leftMeters = targetPose.getY();
                SmartDashboard.putBoolean("TagFollow/HasTarget", true);
                SmartDashboard.putNumber("TagFollow/ForwardMeters", forwardMeters);
                SmartDashboard.putNumber("TagFollow/LeftMeters", leftMeters);

        double forwardCommand = -forwardController.calculate(
                forwardMeters,
                VisionConstants.kTagFollowGoalXMeters);
        double strafeCommand = -strafeController.calculate(
                leftMeters,
                VisionConstants.kTagFollowGoalYMeters);

        double limitedForward = forwardLimiter.calculate(forwardCommand);
        double limitedStrafe = strafeLimiter.calculate(strafeCommand);

        double forwardSpeed = MathUtil.clamp(
                limitedForward,
                -VisionConstants.kTagFollowMaxLinearSpeedMetersPerSecond,
                VisionConstants.kTagFollowMaxLinearSpeedMetersPerSecond);
        double strafeSpeed = MathUtil.clamp(
                limitedStrafe,
                -VisionConstants.kTagFollowMaxLinearSpeedMetersPerSecond,
                VisionConstants.kTagFollowMaxLinearSpeedMetersPerSecond);

        double turningSpeed = 0.0;
        double txRadians = Units.degreesToRadians(LimelightHelpers.getTX(limelightName));
        turningSpeed = aimController.calculate(txRadians, 0.0);
        turningSpeed = MathUtil.clamp(
                turningSpeed,
                -VisionConstants.kAimMaxAngularSpeedRadPerSec,
                VisionConstants.kAimMaxAngularSpeedRadPerSec);

        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(
                forwardSpeed,
                strafeSpeed,
                turningSpeed);
        swerveSubsystem.driveRobotRelative(robotRelativeSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
