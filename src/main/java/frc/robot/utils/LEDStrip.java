package frc.robot.utils;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {
    public AddressableLED led;
    public AddressableLEDBuffer ledBuffer;
    public ArrayList<LEDStrip> followers;

    public LEDStrip(int port, int length) {
        ledBuffer = new AddressableLEDBuffer(length);

        led = new AddressableLED(port);
        led.setLength(ledBuffer.getLength());
        led.start();
    }

    /**
     * follow the contents of another led strip
     *
     * @param led the led strip to follow
     */
    public void followStrip(LEDStrip led) {
        led.followers.add(this);
    }

    /**
     * set led strip and set followers
     *
     * @param strip LEDStrip strip
     */
    public void set() {
        int i;

        led.setData(ledBuffer);
        for (i = 0; i < followers.size(); i++) {
            followers.get(i).led.setData(ledBuffer);
        }
    }

    /**
     * set the buffer content of the LEDStrip to a solid color
     *
     * @param r red
     * @param b green
     * @param g blue
     */
    public void solidColorRGB(int r, int b, int g) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
    }
}
