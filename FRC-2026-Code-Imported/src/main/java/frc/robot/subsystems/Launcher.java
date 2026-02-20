package frc.robot.subsystems;

/** Handles the single-motor launcher used for scoring notes. */

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Launcher extends SubsystemBase {

    private SparkMax motor4;
      private final PIDController elevatorPidController;
        private final RelativeEncoder motor1Encoder;




    /** Sets up the shooter motor, encoder, and PID placeholder. */
    public Launcher() {
        motor4 = new SparkMax(12, MotorType.kBrushless);

        motor1Encoder = motor4.getEncoder();


        elevatorPidController = new PIDController(0.4, 0, 0);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
       
    }

/** Spins the launcher motor at the given percent output. */
public void Spin(double value) {
         SmartDashboard.putNumber("encoder arm", get_encoder());

    motor4.set(-value);
}

/** Stops the launcher motor. */
public void Spin() {
    motor4.set(0);
   
}

/** @return shooter encoder position for logging. */
public double get_encoder(){
    return motor1Encoder.getPosition();
}

}
