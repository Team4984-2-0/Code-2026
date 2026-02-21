package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.resetheading;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoSource;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.Arm;




import frc.robot.commands.Launch;


import frc.robot.commands.Intake;




public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
      
        private final Climber climber = new Climber();
  

        private final Arm arm = new Arm();

        private final Elevator elevator = new Elevator();

        private final Launcher launcher = new Launcher();

        // Limelight rotation controller + rate limiter to prevent oscillation
        private final PIDController limelightPID = new PIDController(0.025, 0.0, 0.0015);
        // Lower rate limit (units/sec) to help eliminate shaking. Tune this between 0.5 and 3.0.
        private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(1.0);

        private final SendableChooser<Command> autoChooser;

        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

        private final XboxController operatorJoytick = new XboxController(OIConstants.kOperatorControllerPort);
        SendableChooser<Command> m_Chooser = new SendableChooser<>();

        public RobotContainer() {
          
                // smooth rotation tolerance
                limelightPID.setTolerance(1.0); // degrees

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);

                //m_Chooser.setDefaultOption("Auto Command", getAutonomousCommand());
                SmartDashboard.putData("Auto Mode",m_Chooser);
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

        private void configureButtonBindings () {
                new JoystickButton(driverJoytick, 2).whileTrue(new resetheading(swerveSubsystem));
                // Launch
             //   new JoystickButton(operatorJoytick, 6).whileTrue(new Launch(launcher));
               // new JoystickButton(operatorJoytick, 7).whileTrue(new Intake(launcher));
                new JoystickButton(driverJoytick, 3).whileTrue(new SwerveJoystickCmd(
                        swerveSubsystem,
                        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                        () -> {
                            // Read tx/tv directly from NetworkTables (limelight)
                            var table = NetworkTableInstance.getDefault().getTable("limelight");
                            double tx = -table.getEntry("tx").getDouble(0.0);
                            double tv = table.getEntry("tv").getDouble(0.0);

                            // if no target, stop auto-rotation and reset smoothing state
                            if (tv < 1.0) {
                                rotRateLimiter.reset(0.0);
                                limelightPID.reset();
                                return 0.0;
                            }

                            // deadband to avoid jitter around center
                            final double DEADBAND_DEG = 1.0;
                            if (Math.abs(tx) <= DEADBAND_DEG) {
                                rotRateLimiter.reset(0.0);
                                limelightPID.reset();
                                return 0.0;
                            }

                            // PID drives tx -> 0.
                            double pidOut = limelightPID.calculate(tx, 0.0); // NOTE: sign flipped here (was negated)

                            // clamp to smaller max to avoid aggressive corrections (reduces shaking)
                            final double MAX_OUTPUT = 0.35;
                            pidOut = Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, pidOut));

                            // smooth sudden changes to avoid oscillation
                            double limited = rotRateLimiter.calculate(pidOut);

                            // Invert final output so positive tx produces the correct rotation direction on this robot
                            return -limited;
                        },
                        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

/*8 
                new JoystickButton(operatorJoytick, 8).whileTrue(new Climb(climber));
                new JoystickButton(operatorJoytick, 7).whileTrue(new Climbbutback(climber));

                new JoystickButton(operatorJoytick, 1).whileTrue(new ElevatorAutoL2(elevator));
               
              // XboxController.Button.
               // new JoystickButton(operatorJoytick, 7).whileTrue(new Launch(launcher)); */
               
        }

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
      
          public void run() {
            System.out.println("CameraThread running");
      
          }
      
          public void setResolutionLow() {
            System.out.println("CameraThread rsetResolutionLow running");
            usbCamera1.setResolution(150, 150);
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
      
          }
      
          public void setResolutionHigh() {
            System.out.println("CameraThread rsetResolutionHigh running");
            usbCamera1.setResolution(150, 150);
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
          }
      
          public void setCameraSource() {
            System.out.println("CameraThread setCameraSource running");
            server.setSource(usbCamera1);
            SmartDashboard.putString("My Key", "Hello");
          }
      
          public void getCameraConfig() {
            System.out.println("CameraThread getPrintCameraConfig running");
            String cameraConfig;
            // issue when camera is not plugged in at start
            cameraConfig = usbCamera1.getConfigJson();
            if (cameraConfig.isEmpty() == false) {
              // System.out.println(cameraConfig.toString()); //print to console
            }
          }
      
          public void setCameraConfig() {
            System.out.println("CameraThread setPrintCameraConfig running");
      
            usbCamera1.setFPS(Constants.CAMERA1_FPS);
            usbCamera1.setBrightness(Constants.CAMERA1_BRIGHTNESS);
            usbCamera1.setExposureAuto();
          }
        }
      }