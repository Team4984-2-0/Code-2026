package frc.robot.subsystems;

/** Winch subsystem reserved for endgame climbing. */

// Removed unused imports (commands.*, LiveWindow, Spark)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {

    private SparkMax climbMotor;
    
     

    /** Initializes the climb motor and idle mode. */
    public Climber() {
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climbMotor = new SparkMax(13, MotorType.kBrushless);
    
    }

/** Spins the climb motor at the provided percent output. */
public void Spin(double value) {
    climbMotor.set(value);
  
}


/** Stops the climb motor. */
public void Spin() {
    climbMotor.set(0);
   
   
}


}
