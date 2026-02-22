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
 * Command-based helper that turns a Limelight AprilTag lock into chassis
 * velocities. While the command is held, it continuously reads the
 * camera-space pose of the detected tag, drives forward/backward until the
 * desired standoff distance is reached, strafes left/right to stay centered,
 * and uses the existing aim PID to keep the tag in the middle of the image.
 */
public class LimelightTagFollow extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final String limelightName;

    /** PID block that regulates distance (camera X axis). */
    private final PIDController forwardController;
    /** PID block that keeps the robot centered on the tag laterally (camera Y axis). */
    private final PIDController strafeController;
    /** Re-uses the aim PID to maintain yaw alignment. */
    private final PIDController aimController;

    /** Slew limiters keep sudden spikes from jolting the drivetrain. */
    private final SlewRateLimiter forwardLimiter;
    private final SlewRateLimiter strafeLimiter;

    /** Convenience ctor that defaults to the primary Limelight name. */
    public LimelightTagFollow(SwerveSubsystem swerveSubsystem) {
        this(swerveSubsystem, VisionConstants.kPrimaryLimelightName);
    }

    /**
     * Creates the command for a specific Limelight instance.
     *
     * @param swerveSubsystem drive base being controlled
     * @param limelightName   NetworkTables name of the camera to query
     */
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
        // Ensure the dashboard reflects that we are currently vision-limited.
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

        // PID controllers operate in camera-space meters: X is forward, Y is left.
        double forwardCommand = -forwardController.calculate(
                forwardMeters,
                VisionConstants.kTagFollowGoalXMeters);
        double strafeCommand = -strafeController.calculate(
                leftMeters,
                VisionConstants.kTagFollowGoalYMeters);

        // Rate-limit the outputs so the chassis does not lurch when the pose jumps.
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
        // Use tx (horizontal pixel error) for yaw control instead of the 3D pose.
        double txRadians = Units.degreesToRadians(LimelightHelpers.getTX(limelightName));
        turningSpeed = aimController.calculate(txRadians, 0.0);
        turningSpeed = MathUtil.clamp(
                turningSpeed,
                -VisionConstants.kAimMaxAngularSpeedRadPerSec,
                VisionConstants.kAimMaxAngularSpeedRadPerSec);
        // Feed robot-relative speeds directly into the subsystem helper.
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
