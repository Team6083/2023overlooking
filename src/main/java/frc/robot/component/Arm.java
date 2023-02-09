package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Arm {
    private static final double encoderPulse = 4096;
    private static final double gearing = 198;
    // rotate arm
    private static CANSparkMax ArmMotor1;
    private static CANSparkMax ArmMotor2;
    private static RelativeEncoder ArmEncoder;
    private static int karm = 0;
    //take up and pay off device
    private static WPI_VictorSPX vic;
    private static final int vic1 =2 ;

    public static void init() {
        ArmMotor1 = new CANSparkMax(karm, MotorType.kBrushless);
        vic = new WPI_VictorSPX(vic1);
    }

    public static void teleop() {
        // get degree position
        double a = positionToDegreeMeter(ArmEncoder.getPosition());
        // rotate arm
        ArmMotor1.set(Robot.xbox.getLeftTriggerAxis());
        ArmMotor2.set(-Robot.xbox.getLeftTriggerAxis());
        ArmMotor1.set(-Robot.xbox.getRightTriggerAxis());
        ArmMotor2.set(Robot.xbox.getRightTriggerAxis());
        //take up and pay off device
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

    public static double autoArm(double speed){
        ArmMotor1.set(speed);
        return 0;
    }

    public  static double autoAccessDegree() {
        double Degree = positionToDegreeMeter(ArmEncoder.getPosition());
        return Degree;
    }
    public static double setVic(double speed){
        vic.set(speed);
        return 0;
    }
}