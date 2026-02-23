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
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.LimelightAutoAim;
import frc.robot.commands.LimelightTagFollow;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.resetheading;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Launcher;

import frc.robot.subsystems.Intake;

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
    // Ported Camera Code
    // SmartDashboard.putData("Auto Mode",m_Chooser);

  }

  /**
   * Houses all button bindings so drivers have a single location to reference
   * when changing control schemes.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverJoytick, 2).whileTrue(new resetheading(swerveSubsystem));
    // new JoystickButton(operatorJoytick, 6).whileTrue(new Launch(launcher));
    // new JoystickButton(operatorJoytick, 7).whileTrue(new Intake(launcher));
    new JoystickButton(driverJoytick, 3).whileTrue(new LimelightAutoAim(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    new JoystickButton(driverJoytick, 4).whileTrue(new LimelightTagFollow(swerveSubsystem));

  }

  /**
   * Returns whichever autonomous routine is currently selected on SmartDashboard.
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();
  }

}