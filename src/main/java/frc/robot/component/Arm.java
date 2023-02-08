package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Arm {
    private static final double encoderPulse = 4096;
    private static final double gearing = 198;
    private static final int vic1 =2 ;
    private static CANSparkMax armmotor;//
    private static RelativeEncoder armencoder;
    private static int karm = 0;
    private static WPI_VictorSPX vic;

    public static void init() {
        armmotor = new CANSparkMax(karm, MotorType.kBrushless);
        vic = new WPI_VictorSPX(vic1);
    }

    public static void teleop() {
        // get degree position
        double a = positionToDegreeMeter(armencoder.getPosition());
        // rotate arm
        armmotor.set(Robot.xbox.getLeftTriggerAxis());
        armmotor.set(-Robot.xbox.getRightTriggerAxis());
        if (Robot.xbox.getPOV() == 0) {
            vic.set(0.5);
        }else if(Robot.xbox.getPOV()==180){
            vic.set(-0.5);
        }else{
            vic.set(0);
        }

    }

    // position calculation
    public static double positionToDegreeMeter(double position) {
        double sensorRate = position / encoderPulse;
        double armRate = sensorRate / gearing;
        double positionMeter = armRate * 360;
        return positionMeter;
    }

    public static double setArm(double speed){
        armmotor.set(speed);
        return 0;
    }

    public  static double accessDegree() {
        double Degree = positionToDegreeMeter(armencoder.getPosition());
        return Degree;
    }
}