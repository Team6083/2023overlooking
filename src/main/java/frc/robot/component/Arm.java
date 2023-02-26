package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Arm {
    // motor
    private static CANSparkMax ArmMotorleft;// rotate arm
    private static CANSparkMax ArmMotorright;
    private static MotorControllerGroup ArmMotor;
    private static final int armL = 15;
    private static final int armR = 16;
    private static double rotate;

    // encoder
    private static Encoder ArmEncoder;
    private static final int armEnc1 = 8;
    private static final int armEnc2 = 9;

    // pid
    private static double kAP = 0.35;
    private static double kAI = 0.0;
    private static double kAD = 0.0;
    private static PIDController ArmPID;
    private static double armAngleModify = 0;

    // value
    private static final double ArmEncoderPulse = 2048;

    public static void init() {
        // motor
        ArmMotorleft = new CANSparkMax(armL, MotorType.kBrushless);
        ArmMotorright = new CANSparkMax(armR, MotorType.kBrushless);
        ArmMotor = new MotorControllerGroup(ArmMotorleft, ArmMotorright);
        ArmMotorleft.setInverted(true);

        // encoder
        ArmEncoder = new Encoder(armEnc1, armEnc2);

        // pid
        ArmPID = new PIDController(kAP, kAI, kAD);
        ArmPID.setSetpoint(68.5);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
    }

    public static void teleop() {
        double angle = positionToDegree(); // get the angular position

        // encoder reset
        if (Robot.xbox.getRawButton(7)) {
            ArmEncoder.reset();
        }

        // adjust P
        kAP = SmartDashboard.getNumber("arm_kP", kAP);
        ArmPID.setP(kAP);

        // rotate arm
        if (Robot.xbox.getXButton()) {
            ArmPID.setSetpoint(35.2);
        } else if (Robot.xbox.getYButton()) {
            ArmPID.setSetpoint(68.5);
        } else {
            armAngleModify = (Robot.xbox.getLeftTriggerAxis() -
                    Robot.xbox.getRightTriggerAxis()) * 0.01;
            ArmPID.setSetpoint(ArmPID.getSetpoint() + armAngleModify);
        }

        // control through xbox, for test 
        rotate = (Robot.xbox.getLeftTriggerAxis() - Robot.xbox.getRightTriggerAxis()) * 0.2;
        ArmMotor.set(rotate);

        Controlloop();

        // put dashboard
        SmartDashboard.putNumber("arm angle", angle);
        SmartDashboard.putNumber("Arm_setpoint", ArmPID.getSetpoint());
        SmartDashboard.putNumber("arm enc", ArmEncoder.get());
    }

    public static void Controlloop() {
        var ArmVolt = ArmPID.calculate(positionToDegree());
        if (Math.abs(ArmVolt) > 10) {
            ArmVolt = 10 * (ArmVolt > 0 ? 1 : -1);
        }
        ArmMotor.setVoltage(ArmVolt);

        SmartDashboard.putNumber("ArmVolt", ArmVolt);
    }

    // do the number of turns calculate(to a particular angle)
    public static double positionToDegree() {
        double armRate = ArmEncoder.get() * 360 / ArmEncoderPulse;
        return armRate;
    }

    // for NewAutoEngine
    public static int autoArmControl(int modeLine, int modeArm) {
        switch (modeArm) {
            case 0:// the beginning position
                ArmPID.setSetpoint(68.5);
                break;
            case 1: // the second level
                ArmPID.setSetpoint(-10);
                break;
            case 2:// the third level
                ArmPID.setSetpoint(35.5);
                break;
            default:
                break;
        }

        switch (modeLine) {
            case 0:// the beginning position
                Line.LinePID.setSetpoint(0);
                break;
            case 2: // the second level
                Line.LinePID.setSetpoint(33.02);
                break;
            case 3:// the third level
                Line.LinePID.setSetpoint(86.15);// the third level
                break;
            default:
                break;
        }

        Controlloop();

        SmartDashboard.putNumber("line enc", Line.LineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line length", Line.positionToLength());
        return 0;
    }
}