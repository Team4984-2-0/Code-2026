package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Launcher;

/**
 * Launches a ball at a speed calculated from the Limelight distance to the
 * currently tracked AprilTag. The command reads the 3-D target pose in
 * camera space every cycle, computes the straight-line distance, maps that
 * distance to a shooter power using a simple linear interpolation between
 * a minimum and maximum range, then runs the hopper and feed rollers once
 * the shooter has had time to spin up.
 *
 * <p>If no target is visible the shooter still runs at a configurable
 * default power so the driver is never left without any output.
 */
public class LimelightLaunch extends Command {

    private final Launcher launcherSubsystem;
    private final String limelightName;

    /** Counts execute() cycles so we can delay the hopper/feed. */
    private int shooterCounter = 0;

    /** Cycle counts before hopper and feed engage (same cadence as Launch). */
    private static final int START_HOPPER_COUNT = 75;
    private static final int START_FEED_COUNT   = 75;

    /** The shooter power we computed at the start of this press. */
    private double targetShooterPower = VisionConstants.kLaunchDefaultPower;

    public LimelightLaunch(Launcher launcherSubsystem) {
        this(launcherSubsystem, VisionConstants.kPrimaryLimelightName);
    }

    public LimelightLaunch(Launcher launcherSubsystem, String limelightName) {
        this.launcherSubsystem = launcherSubsystem;
        this.limelightName = limelightName;
        addRequirements(launcherSubsystem);
    }

    @Override
    public void initialize() {
        shooterCounter = 0;
        targetShooterPower = computeShooterPower();
        //SmartDashboard.putNumber("LimelightLaunch/Power", targetShooterPower);
    }

    @Override
    public void execute() {
        // Recalculate power each cycle so it tracks if the robot is still moving
        targetShooterPower = computeShooterPower();
        SmartDashboard.putNumber("LimelightLaunch/Power", targetShooterPower);

        launcherSubsystem.ShootRun(targetShooterPower);
        shooterCounter++;

        if (shooterCounter > START_HOPPER_COUNT) {
            launcherSubsystem.HopperRun(0.2);
        }
        if (shooterCounter > START_FEED_COUNT) {
            launcherSubsystem.FeedRun(-0.80);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterCounter = 0;
        launcherSubsystem.Killswitch();
    }

    // ---- helpers ----

    /**
     * Uses the Limelight 3-D target pose (camera space) to compute the
     * straight-line distance to the AprilTag, then maps that distance to a
     * shooter motor power.
     *
     * <p>The mapping is a simple linear interpolation clamped between the
     * configured min/max distance and power values in
     * {@link VisionConstants}.
     *
     * @return motor power in the range
     *         [{@code kLaunchMinPower}, {@code kLaunchMaxPower}]
     */
    private double computeShooterPower() {
        if (!LimelightHelpers.getTV(limelightName)) {
            SmartDashboard.putNumber("LimelightLaunch/DistanceM", -1);
            SmartDashboard.putBoolean("LimelightLaunch/HasTarget", false);
            return VisionConstants.kLaunchDefaultPower;
        }

        Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
        double x = targetPose.getX(); // forward distance
        double y = targetPose.getY(); // lateral distance
        double z = targetPose.getZ(); // vertical distance

        // Euclidean distance from camera to tag
        double distanceMeters = Math.sqrt(x * x + y * y + z * z);

        SmartDashboard.putNumber("LimelightLaunch/DistanceM", distanceMeters);
        SmartDashboard.putBoolean("LimelightLaunch/HasTarget", true);

        // Linear interpolation: farther away → more power
        double t = (distanceMeters - VisionConstants.kLaunchMinDistanceMeters)
                 / (VisionConstants.kLaunchMaxDistanceMeters - VisionConstants.kLaunchMinDistanceMeters);
        t = MathUtil.clamp(t, 0.0, 1.0);

        double power = VisionConstants.kLaunchMinPower
                      + t * (VisionConstants.kLaunchMaxPower - VisionConstants.kLaunchMinPower);

        return power;
    }
}
