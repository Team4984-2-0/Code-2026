package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Winch subsystem reserved for endgame climbing. */

// Removed unused imports (commands.*, LiveWindow, Spark)
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
    private final RelativeEncoder motor1Encoder;
    private SparkMax climbMotor;

    /** Initializes the climb motor and idle mode. */
    public Climber() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climbMotor = new SparkMax(9, MotorType.kBrushless);
        motor1Encoder = climbMotor.getEncoder();

    }

    /** Spins the climb motor at the provided percent output, with safety and readjustment logic. */
    public void Spin(double value) {
        double position = motor1Encoder.getPosition();
        double effectiveValue = value;
        double maximum = 1.49;
        double minimum = 0;


        if (position < 0 && effectiveValue > 0) {climbMotor.set(effectiveValue);}
        else if (position > maximum && effectiveValue < 0) {climbMotor.set(effectiveValue);}
        else if (position < maximum && position > minimum) {climbMotor.set(effectiveValue);}
        else {
            climbMotor.set(0);
        }

        //climbMotor.set(effectiveValue);
        SmartDashboard.putNumber("encoder elevator", get_encoderClimb());
    }
    public void manSpin(double value) {
      
            climbMotor.set(value);


        //climbMotor.set(effectiveValue);
        SmartDashboard.putNumber("encoder elevator", get_encoderClimb());
    }

    /** Stops the climb motor. */
    public void Spin() {
        
        climbMotor.set(0);

    }
public double get_encoderClimb(){
    return motor1Encoder.getPosition();
}

}
