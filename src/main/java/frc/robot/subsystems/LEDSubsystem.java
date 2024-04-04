package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LEDStrip;

public class LEDSubsystem extends SubsystemBase {
    public LEDStrip strip0;
    public LEDStrip strip1;
    public Spark blinkin;

    public LEDSubsystem() {
        blinkin = new Spark(0);
        //strip0 = new LEDStrip(5, 20);
        // strip1 = new LEDStrip(1, 27);

        // strip1.followStrip(strip0);
    }

    public void setGreen() {
        blinkin.set(0.77);
    }
    
    public void setDefault(){
        //0.11 = color1, 0.31 = color2
        blinkin.set(Constants.Robot.isRedAlliance ? -0.31 : -0.29);
    }

    public void setForestTinkle(){
        blinkin.set(-0.47);
    }

    public void setFire(){
        blinkin.set(-0.57);
    }
}