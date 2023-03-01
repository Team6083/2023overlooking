package frc.robot.component;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Light {
    public static AddressableLED led;
    public static AddressableLEDBuffer ledBuffer;
    private static final int ledPort = 0; // PWM port to be confirmed
    private static final int ledLength = 10; // to be confirmed
    private static int firstHue = 0;
    private static int mode = 0;

    public static void init() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength); // set length
        led.setLength(ledBuffer.getLength());
        led.start();
        led.setData(ledBuffer);
    }

    public static void teleop() {
        if (Robot.viceController.getXButtonPressed()) {
            mode = (mode == 1 ? 2 : 1);
        } else if (Robot.viceController.getAButtonPressed()) {
            mode = 0;
        }
        switch (mode) {
            case 0:
                rainbow();
                break;
            case 1: // purple cube
                setHSVLoop(80, 255, 127);
                break;
            case 2: // yellow cone
                setHSVLoop(300, 255, 127);
                break;
            default:
        }
        led.setData(ledBuffer);
        putDashboard();
    }

    public static void disabledInit() {
        led.stop();
    }

    public static void setHSVLoop(int h, int s, int v) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, h, s, v);
        }
    }

    public static void rainbow() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            var hue = (firstHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 127);
        }
        firstHue += 3;
        firstHue %= 180;
    }

    public static void putDashboard() {
        switch (mode) {
            case 0:
                SmartDashboard.putString("LED_color", "none");
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
