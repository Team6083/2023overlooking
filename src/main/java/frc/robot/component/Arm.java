package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Arm {
    private static final double encoderPulse = 4096;
    private static final double gearing = 198;
    private static CANSparkMax armmotor ;//
    private static RelativeEncoder armencoder;
    private static int karm = 0;


    public static void init() {
    armmotor = new CANSparkMax(karm, MotorType.kBrushless);
    }
    public static void teleop() {
        //get degree position
        double a = positionToDegreeMeter(armencoder.getPosition());
        //
        armmotor.set(Robot.xbox.getLeftTriggerAxis());
        armmotor.set(-Robot.xbox.getRightTriggerAxis());
    } 
    public static double positionToDegreeMeter(double position){
            double sensorRate = position/encoderPulse;
            double armRate = sensorRate/gearing;
            double positionMeter=armRate*360; 
            return positionMeter;
        }
}