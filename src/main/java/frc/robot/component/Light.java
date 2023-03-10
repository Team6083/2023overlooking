package frc.robot.component;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Light {
    public static DigitalOutput redLight;
    public static DigitalOutput greenLight;
    public static DigitalOutput blueLight;
    private static Timer time;
    private static int lightMode = 0;

    public static void init() {
        redLight = new DigitalOutput(2); // red dio channel
        greenLight = new DigitalOutput(0); // green dio channel
        blueLight = new DigitalOutput(1); // blue dio channel
        time = new Timer();
        time.start();
    }

    public static void teleop() {
        if (Robot.viceController.getXButtonPressed()) {
            lightMode = (lightMode == 1 ? 2 : 1);
        } else if (Robot.viceController.getAButtonPressed()) {
            lightMode = 0;
        }
        switch (lightMode) {
            case 0:
                free();
                break;
            case 1: // purple cube
                setLight(false, true, false);
                break;
            case 2: // yellow cone
                setLight(false, false, true);
                break;
        }
        putDashboard();

    }

    public static void disabledInit() {
        setLight(true, true, true);
    }

    public static void free() {
        SmartDashboard.putNumber("timer", time.get());
        if (time.get() > 0.25) {
            int freeMode = (int) (Math.random() * 3);
            SmartDashboard.putNumber("freemode", freeMode);
            switch (freeMode) {
                case 0:
                    setLight(!redLight.get(), greenLight.get(), blueLight.get());
                    break;
                case 1:
                    setLight(redLight.get(), !greenLight.get(), blueLight.get());
                    break;
                case 2:
                    setLight(redLight.get(), greenLight.get(), !blueLight.get());
                    break;
            }
            time.reset();
        }
    }

    public static void setLight(boolean r, boolean g, boolean b) {
        redLight.set(r);
        greenLight.set(g);
        blueLight.set(b);
    }

    public static void putDashboard() {
        switch (lightMode) {
            case 0:
                SmartDashboard.putString("LED_color", "free");
                break;
            case 1:
                SmartDashboard.putString("LED_color", "purple_cube");
                break;
            case 2:
                SmartDashboard.putString("LED_color", "yellow_cone");
                break;
            default:
        }
    }
}
