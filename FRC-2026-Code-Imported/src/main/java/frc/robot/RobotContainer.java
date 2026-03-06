package frc.robot;

/**
 * Central wiring for the robot. This class connects operator controls,
 * subsystems, default commands, and autonomous routines so the rest of the
 * codebase can stay focused on single responsibilities.
 */

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.LimelightAutoAim;
import frc.robot.commands.LimelightTagFollow;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.resetheading;
import frc.robot.commands.SetLimelightPipeline0;
import frc.robot.commands.SetLimelightPipeline1;
import frc.robot.commands.SetLimelightPipeline2;
import frc.robot.commands.SetLimelightPipeline3;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Launcher;

import frc.robot.subsystems.Intake;
import java.util.Timer;
import java.util.TimerTask;

public class RobotContainer {

  /** Primary swerve drivebase used in both teleop and auto. */
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** Placeholder climber subsystem (bindings can be re-enabled later). */
  private final Climber climber = new Climber();

  /** Articulated arm for scoring/collecting pieces. */
  private final Intake intake = new Intake();

  /** Launcher/shooter for game-specific notes. */
  private final Launcher launcher = new Launcher();

  /** Shuffleboard chooser for PathPlanner-generated autos. */
  private final SendableChooser<Command> autoChooser;

  // joystick and xbox controller for driver and operator, respectively. Port
  // numbers are defined in OIConstants.
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  private final XboxController operatorJoytick = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  /**
   * Builds the container, default command bindings, and starts the auxiliary
   * camera thread used for driver vision.
   */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // m_Chooser.setDefaultOption("Auto Command", getAutonomousCommand());
    SmartDashboard.putData("Auto Mode", m_Chooser);
    // Default teleop command: map joystick axes to field-oriented drive inputs.
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    // launcher.setDefaultCommand(new Shooter(launcher));
    configureButtonBindings();

    // Publish simple boolean for Shuffleboard (true = tag seen)
    Timer limelightTimer = new Timer(true);
    limelightTimer.scheduleAtFixedRate(new TimerTask() {
      @Override
      public void run() {
        var table = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = table.getEntry("tv").getDouble(0.0);   // 1 = target visible
        double tid = table.getEntry("tid").getDouble(-1); // tag id if present
        boolean hasTag = (tv >= 1.0) || (tid >= 0.0);
        SmartDashboard.putBoolean("Limelight/HasTag", hasTag);
      }
    }, 0, 200);

    // Ported Camera Code
    // SmartDashboard.putData("Auto Mode",m_Chooser);

  }

  // Helper to get current AprilTag ID from Limelight
  private double getTid() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1);
  }

  private void configureButtonBindings() {

    //////////////////////////////// Driver Controller ////////////////////////////////
    /// 
    // Reset Heading
       new JoystickButton(driverJoytick, 2).whileTrue(new resetheading(swerveSubsystem));

   // Automatically Aiming with Limelight
       new JoystickButton(driverJoytick, 3).whileTrue(new LimelightAutoAim(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    
    // Automatically Driving with Limelight
        new JoystickButton(driverJoytick, 4).whileTrue(new LimelightTagFollow(swerveSubsystem));




    //////////////////////////////// Operator Controller ////////////////////////////////

    // Climber
       new JoystickButton(operatorJoytick, 2).whileTrue(new Climb(climber));
       new JoystickButton(operatorJoytick, 1).whileTrue(new ClimbDown(climber));

    // Line Up with Climber: only if AprilTag 16 is visible
       Trigger tag16Visible = new Trigger(() -> getTid() == 16);
       tag16Visible.and(new JoystickButton(operatorJoytick, 3)).whileTrue(new LimelightAutoAim(
        swerveSubsystem,
        () -> -operatorJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -operatorJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> !operatorJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
       tag16Visible.and(new JoystickButton(operatorJoytick, 3)).whileTrue(new LimelightTagFollow(swerveSubsystem));


    // Swapping Diffrent LimeLight PipeLines
       // new JoystickButton(operatorJoytick, 1).whileTrue(new Climb(climber));
 
    // new JoystickButton(operatorJoytick, 6).whileTrue(new Launch(launcher));
    // new JoystickButton(operatorJoytick, 7).whileTrue(new Intake(launcher));
    

  }
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();
  }

}