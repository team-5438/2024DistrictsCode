package frc.robot.utils;

public class FlashLEDS extends Thread {
    private int ar[][];
    private LEDStrip led;

    private Thread t;

    public FlashLEDS(LEDStrip led, int ar[][]) {
        this.ar = ar;
        this.led = led;

        t = new Thread(this, "LED Thread");
        t.start();
    }

    @Override
    public void run() {
        int r, g, b, time = 0;

        for (int i = 0; i <= ar.length; i++) {
            try {
                Thread.sleep(time);
            } catch (InterruptedException e) {
                System.out.println("Failed to sleep");
                e.printStackTrace();
            }

            r = ar[i][0];
            g = ar[i][1];
            b = ar[i][2];
            time = ar[i][3];

            led.solidColorRGB(r, g, b);
            led.set();
        }
    }
}