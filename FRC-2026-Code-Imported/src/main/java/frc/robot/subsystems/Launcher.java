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
      private final RelativeEncoder motor2Encoder;
        private final RelativeEncoder motor1Encoder;
        private final RelativeEncoder motor3Encoder;
        private SparkMax Hopper; 
        private SparkMax Feed;
        private SparkMax Shoot;




    /** Sets up the shooter motor, encoder, and PID placeholder. */
    public Launcher() {
        Hopper = new SparkMax(12, MotorType.kBrushless);
        Feed = new SparkMax(13, MotorType.kBrushless);
        Shoot = new SparkMax(14, MotorType.kBrushless);
        // Device ids may need a change in number

        motor1Encoder = Hopper.getEncoder();

        motor2Encoder = Feed.getEncoder();
        
        motor3Encoder = Shoot.getEncoder();
        

        

        
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
       
    }



public void Killswitch() {
    Feed.set(0);
    Hopper.set(0);
    Shoot.set(0);
   
}

/** @return shooter encoder position for logging. */
public double get_encoder(){
    return motor2Encoder.getPosition();
}
public void HopperRun(double value) {
    Hopper.set(-value);

}
public void FeedRun(double value){
    Feed.set(-value);
}
public void ShootRun(double value) {
    Shoot.set(-value);
}
public void HopperStop() {
    Hopper.set(0);
}
public void FeedStop() {
    Feed.set(0);
}
public void ShootStop() {
    Feed.set(0);
}
}
