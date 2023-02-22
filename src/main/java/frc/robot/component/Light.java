package frc.robot.component;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;

public class Light {
    public static AddressableLED led;
    public static AddressableLEDBuffer ledBuffer;
    private static final Color purple = new Color(255, 0, 255);
    private static final Color yellow = new Color(255, 255, 0);
    private static final int ledPort = 0; // PWM port to be confirmed
    private static final int ledLength = 10; // to be confirmed

    public static void init() {
        led = new AddressableLED(ledPort);
        ledBuffer = new AddressableLEDBuffer(ledLength); // set length
        led.setLength(ledBuffer.getLength());
        led.start();
        led.setData(ledBuffer);
    }

    public static void teleop() {
        if (Robot.xbox.getAButtonPressed()) { // buttons to be confirmed
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 0, 255); // purple
            }
        }
        if (Robot.xbox.getBButtonPressed()) { // buttons to be confirmed
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, 255, 255, 0); // yellow
            }
        }
        led.setData(ledBuffer);
        putDashboard();
    }

    public static void disabledInit() {
        led.stop();
    }

    public static void putDashboard() {
        if (ledBuffer.getLED(0) == purple) {
            SmartDashboard.putString("LED color", "purple cube");
        } else if (ledBuffer.getLED(0) == yellow) {
            SmartDashboard.putString("LED color", "yellow cone");
        }
    }
}
