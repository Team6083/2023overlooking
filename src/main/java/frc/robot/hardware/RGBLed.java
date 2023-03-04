package frc.robot.hardware;

import edu.wpi.first.wpilibj.DigitalOutput;

public class RGBLed {
    private final DigitalOutput doRed, doGreen, doBlue;

    private boolean redOn, greenOn, blueOn;

    public RGBLed(int redCh, int greenCh, int blueCh) {
        doRed = new DigitalOutput(redCh);
        doGreen = new DigitalOutput(greenCh);
        doBlue = new DigitalOutput(blueCh);

        redOn = false;
        greenOn = false;
        blueOn = false;
    }

    public boolean[] get() {
        boolean[] arr = new boolean[3];
        arr[0] = redOn;
        arr[1] = greenOn;
        arr[2] = blueOn;

        return arr;
    }

    public void set(boolean red, boolean green, boolean blue) {
        redOn = red;
        greenOn = green;
        blueOn = blue;
        update();
    }

    private void update() {
        doRed.set(!redOn);
        doBlue.set(!blueOn);
        doGreen.set(!greenOn);
    }
}
