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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
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

        // joystick and xbox controller for driver and operator, respectively. Port numbers are defined in OIConstants.
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

                //m_Chooser.setDefaultOption("Auto Command", getAutonomousCommand());
                SmartDashboard.putData("Auto Mode",m_Chooser);
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
                //SmartDashboard.putData("Auto Mode",m_Chooser);
                CameraThread myCameraThread = null;

                try {
                        myCameraThread = new CameraThread();
                        CameraServer.getServer("test");
                        // CameraServer.startAutomaticCapture();
                        usbCamera1 = CameraServer.startAutomaticCapture(myCameraThread.CAMERA1);
                        // usbCamera2 = CameraServer.startAutomaticCapture(myCameraThread.CAMERA2);
                        // CameraServer.getServer();
                        myCameraThread.server = CameraServer.getServer();
                        usbCamera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
                        myCameraThread.setCameraConfig();

                        myCameraThread.start();
                        myCameraThread.setResolutionHigh();
                        // myCameraThread.getCameraConfig();
                } finally {
                        myCameraThread = null;
                }
        }

  /**
   * Houses all button bindings so drivers have a single location to reference
   * when changing control schemes.
   */
  private void configureButtonBindings () {
                new JoystickButton(driverJoytick, 2).whileTrue(new resetheading(swerveSubsystem));
                //new JoystickButton(operatorJoytick, 6).whileTrue(new Launch(launcher));
                //new JoystickButton(operatorJoytick, 7).whileTrue(new Intake(launcher));
                new JoystickButton(driverJoytick, 3).whileTrue(new LimelightAutoAim(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
                new JoystickButton(driverJoytick, 4).whileTrue(new LimelightTagFollow(swerveSubsystem));
          
        }

        /** Returns whichever autonomous routine is currently selected on SmartDashboard. */
          public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
       // to first load your paths/autos when code starts, then return the
       // pre-loaded auto/path
       return autoChooser.getSelected();
       }
        public static UsbCamera usbCamera1 = null;

        // public static UsbCamera usbCamera2 = null;
        public class CameraThread extends Thread {
          final int CAMERA1 = 0;
          // final int CAMERA2 = 1;
          private final int currentCamera = CAMERA1; // UNCOMMENT WHEN RUNNING THE PROGRAM THRU ROBORIO!!!!
      
          VideoSink server;
      
          /** Keeps camera capture alive; runs after start() */
          public void run() {
            System.out.println("CameraThread running");
      
          }
      
          /** Driver-station helper to lower cost of the stream if bandwidth limited. */
          public void setResolutionLow() {
            System.out.println("CameraThread rsetResolutionLow running");
            usbCamera1.setResolution(150, 150);
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
      
          }
      
          /** Driver-station helper to yield crisper vision when bandwidth allows. */
          public void setResolutionHigh() {
            System.out.println("CameraThread rsetResolutionHigh running");
            usbCamera1.setResolution(150, 150);
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
          }
      
          /** Resets the current USB feed into the server, useful when unplugging cameras. */
          public void setCameraSource() {
            System.out.println("CameraThread setCameraSource running");
            server.setSource(usbCamera1);
            SmartDashboard.putString("My Key", "Hello");
          }
      
          /** Dumps the active JSON configuration string for troubleshooting. */
          public void getCameraConfig() {
            System.out.println("CameraThread getPrintCameraConfig running");
            String cameraConfig;
            // issue when camera is not plugged in at start
            cameraConfig = usbCamera1.getConfigJson();
            if (cameraConfig.isEmpty() == false) {
              // System.out.println(cameraConfig.toString()); //print to console
            }
          }
      
          /** Applies the stored FPS, exposure, and brightness defaults. */
          public void setCameraConfig() {
            System.out.println("CameraThread setPrintCameraConfig running");
      
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
            usbCamera1.setBrightness(Constants.CAMERA1_BRIGHTNESS);
            usbCamera1.setExposureAuto();
          }
        }
      }