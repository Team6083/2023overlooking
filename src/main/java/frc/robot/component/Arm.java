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
    // rotate arm
    private static CANSparkMax ArmMotor;
    private static RelativeEncoder ArmEncoder;
    private static int karm = 0;
    //take up and pay off device
    private static WPI_VictorSPX vic;
    private static final int vic1 =2 ;
    public static void init() {
        ArmMotor = new CANSparkMax(karm, MotorType.kBrushless);
        vic = new WPI_VictorSPX(vic1);
    }

    public static void teleop() {
        // get degree position
        double a = positionToDegreeMeter(ArmEncoder.getPosition());
        // rotate arm
        ArmMotor.set(Robot.xbox.getLeftTriggerAxis());
        ArmMotor.set(-Robot.xbox.getRightTriggerAxis());
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

    public static double setArm(double speed){
        ArmMotor.set(speed);
        return 0;
    }

    public  static double accessDegree() {
        double Degree = positionToDegreeMeter(ArmEncoder.getPosition());
        return Degree;
    }
}