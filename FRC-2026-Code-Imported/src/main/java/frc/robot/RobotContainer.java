package frc.robot;

/**
 * Central wiring for the robot. This class connects operator controls,
 * subsystems, default commands, and autonomous routines so the rest of the
 * codebase can stay focused on single responsibilities.
 */

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Timer;
import java.util.TimerTask;

////////////////////Commands and subsystems////////////////////

//////////Driver//////////

//Driver Speed
import frc.robot.commands.ChangeSpeedDanger;
import frc.robot.commands.ChangeSpeedRegular;

//Swerve
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.resetheading;

//Limelight
import frc.robot.commands.LimelightAutoAim;

//////////Operator//////////

//Climber
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbDownMan;
import frc.robot.commands.AutoClimb;

//Intake & Launcher
import frc.robot.commands.Launch;
import frc.robot.commands.thingspin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;
import frc.robot.commands.RollerOn;
import frc.robot.commands.Cleanup;

// Intake Arm
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmUp;


//Limelight
import frc.robot.commands.AlignToTagCustomValues;
import frc.robot.commands.LimelightLaunch;
import frc.robot.commands.AutoLaunch;


public class RobotContainer {

  /** Primary swerve drivebase used in both teleop and auto. */
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  /** Placeholder climber subsystem (bindings can be re-enabled later). */
  private final Climber climber = new Climber();

  /** Articulated arm for scoring/collecting pieces. */

    private final Intake intake = new Intake();

    public double batteryVoltage = 0.0;

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

    NamedCommands.registerCommand("AutoShoot", new AutoLaunch(launcher));


    SmartDashboard.putNumber("Battery Volage", batteryVoltage);

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
   
    // Change Speed
       new JoystickButton(driverJoytick, 3).whileTrue(new ChangeSpeedDanger(swerveSubsystem));
       new JoystickButton(driverJoytick, 4).whileTrue(new ChangeSpeedRegular(swerveSubsystem));
   
    // Reset Heading
       new JoystickButton(driverJoytick, 2).whileTrue(new resetheading(swerveSubsystem));

   // Automatically Aiming with Limelight
       new JoystickButton(driverJoytick, 1).whileTrue(new LimelightAutoAim(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    
    // Automatically Driving with Limelight
    
//best code

    //////////////////////////////// Operator Controller ////////////////////////////////

    //Launcher
        new JoystickButton(operatorJoytick, 8).whileTrue(new Launch(launcher));
        new JoystickButton(operatorJoytick, 5).whileTrue(new Cleanup(launcher));

       new JoystickButton(operatorJoytick, 6).whileTrue(new LimelightLaunch(launcher));




    //Intake
        new JoystickButton(operatorJoytick, 9).whileTrue(new RollerOn(intake));
             new JoystickButton(operatorJoytick, 9).whileTrue(new thingspin(launcher));

    //Arm
        new JoystickButton(operatorJoytick, 11).whileTrue(new ArmDown(intake));
        new JoystickButton(operatorJoytick, 10).whileTrue(new ArmUp(intake));
   

    // Climber
    new JoystickButton(operatorJoytick, 12).toggleOnTrue(new AutoClimb(swerveSubsystem));   
    new JoystickButton(operatorJoytick, 12).toggleOnTrue(new Climb(climber));



       new JoystickButton(operatorJoytick, 1).whileTrue(new Climb(climber));
       new JoystickButton(operatorJoytick, 2).whileTrue(new ClimbDown(climber));
       new JoystickButton(operatorJoytick, 3).whileTrue(new ClimbDownMan(climber));


    new JoystickButton(operatorJoytick, 7).whileTrue(
        new AlignToTagCustomValues(swerveSubsystem, 11, -22.77, -1.11, 2.053)
    );
  }
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();

  }
}
