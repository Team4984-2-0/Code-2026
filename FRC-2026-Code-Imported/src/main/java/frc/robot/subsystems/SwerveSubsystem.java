package frc.robot.subsystems;

/**
 * Owns the four swerve modules, NavX heading source, and all odometry/helper
 * utilities required for field-relative driving and PathPlanner autos.
 */

import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.pathplanner.lib.config.PIDConstants;
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);


    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI); //AHRS(SPI.Port.kMXP);

    //private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final SwerveModule[] modules = {frontLeft,frontRight,backLeft,backRight};

    

  /** Field visualization published to Shuffleboard. */
  private Field2d myfield;
    private SwerveModulePosition frontleftpos = new SwerveModulePosition (frontLeft.getDrivePosition(),frontLeft.getState().angle);
    private SwerveModulePosition frontrightpos = new SwerveModulePosition (frontRight.getDrivePosition(),frontRight.getState().angle);
    private SwerveModulePosition backLeftpos = new SwerveModulePosition (backLeft.getDrivePosition(),backLeft.getState().angle);
    private SwerveModulePosition backRightpos = new SwerveModulePosition (backRight.getDrivePosition(),backRight.getState().angle);

    private SwerveModulePosition[] WheelPositions = {frontleftpos,frontrightpos,backLeftpos,backRightpos};
    
  
    double xLocationStartFeet = 11.495;   //down field long 
    double yLocationStartFeet = 3.897;  // side to side 
    Rotation2d angleStartDegrees = Rotation2d.fromDegrees(0.0);
    Pose2d startingPosition = new Pose2d(xLocationStartFeet, yLocationStartFeet, angleStartDegrees);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), WheelPositions, startingPosition);
    private SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
    RobotConfig configs;
  /** Configures modules, odometry, and auto builder hooks. */
  public SwerveSubsystem() {
        
      // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
      
    try{
      configs = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            configs, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders(); 
        myfield = new Field2d();
    }

    public void maxspeed(int speed) {
        frontLeft.set_speed(speed);
        frontRight.set_speed(speed);
        backLeft.set_speed(speed);
        backRight.set_speed(speed);
    
    }
    
      /** Hard-resets odometry to a supplied pose (used when a path starts elsewhere). */
      public void resetPose(Pose2d pose) {
        System.out.println(pose);
        odometer.resetPosition(gyro.getRotation2d(), getPositions(), pose);
      }
    
      /** Returns current chassis speeds derived from module states. */
      public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
      }
    
      public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
      }
    
      /** Core drive helper used by both teleop and AutoBuilder. */
      public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        //setStates(targetStates);
        myfield.setRobotPose(odometer.getPoseMeters());
        setModuleStates(targetStates);
      }
    
      /** Convenience wrapper that also updates the field widget. */
      public void setStates(SwerveModuleState[] targetStates) {
        myfield.setRobotPose(odometer.getPoseMeters());
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
        for (int i = 0; i < modules.length; i++) {
          modules[i].setDesiredState(targetStates[i]);
        }
      }
    
      public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
          states[i] = modules[i].getState();
        }
        return states;
      }
      
      public SwerveModulePosition[] getPositions() {
        return WheelPositions;
      }

  /** Zeroes the NavX yaw reading (used when drivers press reset heading). */
  public void zeroHeading() {
        gyro.reset();
    }

  /** Returns the wrapped yaw angle from the NavX so values stay within +/-180. */
  public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
        
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

  /** Latest fused pose from WPILib odometry. */
  public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetpose(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), WheelPositions, pose);
    }

  @Override
  public void periodic() {
        WheelPositions[0].angle =  frontLeft.getState().angle;
        WheelPositions[0].distanceMeters =  frontLeft.getDrivePosition();

        WheelPositions[1].angle =  frontRight.getState().angle;
        WheelPositions[1].distanceMeters =  frontRight.getDrivePosition();

        WheelPositions[2].angle =  backLeft.getState().angle;
        WheelPositions[2].distanceMeters =  backLeft.getDrivePosition();

        WheelPositions[3].angle =  backRight.getState().angle;
        WheelPositions[3].distanceMeters =  backRight.getDrivePosition();
        

        odometer.update(getRotation2d(), WheelPositions);
        SmartDashboard.putData("Field",myfield);
        myfield.setRobotPose(odometer.getPoseMeters());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("odometer X", getPose().getX());
        SmartDashboard.putNumber("odometer Y", getPose().getY());
    }
    
  /** Immediately halts all swerve modules. */
  public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

  /** Applies desired module states after desaturating wheel speeds. */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
        //TBD.NeedsARealFixButEliminatesCompileErrors
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}