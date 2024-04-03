package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.LEDStrip;

public class LEDSubsystem extends SubsystemBase {
    public LEDStrip strip0;
    public LEDStrip strip1;

    public LEDSubsystem() {
        strip0 = new LEDStrip(0, 20);
        // strip1 = new LEDStrip(1, 27);

        // strip1.followStrip(strip0);
    }
}