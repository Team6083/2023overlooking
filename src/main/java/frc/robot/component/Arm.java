package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Robot;

public class Arm {
    private static final double ArmencoderPulse = 4096;
    private static final double Armgearing = 198;
    private static final double lineencoderPulse = 8192;
    private static final double linegearing = 64;

    // rotate arm
    private static CANSparkMax ArmMotor1;
    private static CANSparkMax ArmMotor2;
    private static final int karm = 0;
    private static RelativeEncoder ArmEncoder;
    // private static SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    
    //take up and pay off device
    private static WPI_VictorSPX line;
    private static final int kline =2 ;
    private static Encoder lineEncoder;

    public static void init() {
        ArmMotor1 = new CANSparkMax(karm, MotorType.kBrushless);
        ArmMotor2 = new CANSparkMax(karm, MotorType.kBrushless);
        ArmEncoder= ArmMotor1.getEncoder();
        //ArmEncoder= ArmMotor1.getAlternateEncoder(kAltEncType, kCPR);
        line = new WPI_VictorSPX(kline);
        lineEncoder = new Encoder(0, 1);
    }

    public static void teleop() {
        // rotate arm
        ArmMotor1.set(Robot.xbox.getLeftTriggerAxis());
        ArmMotor2.set(-Robot.xbox.getLeftTriggerAxis());
        ArmMotor1.set(-Robot.xbox.getRightTriggerAxis());
        ArmMotor2.set(Robot.xbox.getRightTriggerAxis());

        //take up and pay off device
        if (Robot.xbox.getPOV() == 0) {
            line.set(0.5);
        } else if (Robot.xbox.getPOV() == 180) {
            line.set(-0.5);
        } else {
            line.set(0);
        }

        // get the angular position
        double angle = positionToDegreeMeter(ArmEncoder.getPosition());

        // get length position
        double length = positionTolengthMeter(lineEncoder.get());

        if(Robot.xbox.getAButton()){
            if(angle>35){
                ArmMotor1.set(-0.5);
                ArmMotor2.set(0.5);
            } else if (angle<35){
                ArmMotor1.set(0.5);
                ArmMotor2.set(-0.5);
            } else if (angle==35){
                ArmMotor1.set(0);
                ArmMotor2.set(0);
            }

            if (length>186){
                line.set(-0.5);
            } else if (length<186){
                line.set(0.5);
            } else if (length==186){
                line.set(0);
            }
        }
    }

    // do the number of turns calculate(to a particular angle)
    public static double positionToDegreeMeter(double turns ) {
        double sensorRate = turns / ArmencoderPulse;
        double armRate = sensorRate / Armgearing;
        double DegreeMeter = armRate * 360;
        return DegreeMeter;
    }

    // do the number of turns calculate(to a particular length)
    public static double positionTolengthMeter(double position) {
        double sensorRate = position / lineencoderPulse;
        double armRate = sensorRate / linegearing;
        double lengthMeter = armRate;
        return lengthMeter;
    }

    public static double autoArm(double speed){
        ArmMotor1.set(speed);
        return 0;
    }

    public static double autoAccessDegree() {
        double Degree = positionToDegreeMeter(ArmEncoder.getPosition());
        return Degree;
    }

    public static double autoLine(double speed){
        line.set(speed);
        return 0;
    }
}