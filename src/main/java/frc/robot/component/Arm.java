package frc.robot.component;

public class Arm {
    public static final double encoderPulse = 4096;
    public static final double gearing = 512;
    public static void init() {}
    public static void teleop() {} 
    public static double positionToDistanceMeter(double position){
            double sensorRate = position/encoderPulse;
            double wheelRate = sensorRate/gearing;
            double positionMeter=wheelRate*360; 
            return positionMeter;
        }
}