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
    private static CANSparkMax armMotorleft; // rotate arm
    private static CANSparkMax armMotorright;
    private static MotorControllerGroup armMotor;
    private static final int armLeftCANId = 15;
    private static final int armRightCANId = 16;

    // encoder
    private static Encoder armEncoder;
    private static final int armEnc1Channel = 8;
    private static final int armEnc2Channel = 9;

    // arm pid
    private static double kAP = 0.35;
    private static double kAI = 0.0;
    private static double kAD = 0.0;
    private static PIDController armPID;

    // value
    private static final double armEncoderPulse = 2048;
    private static final double armVoltLimit = 10;
    private static final double armAngleMin = -30;
    private static final double armAngleMax = 210;

    public static void init() {
        // motor
        armMotorleft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorright = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorleft, armMotorright);
        armMotorleft.setInverted(true);

        // encoder
        armEncoder = new Encoder(armEnc1Channel, armEnc2Channel);

        // pid
        armPID = new PIDController(kAP, kAI, kAD);
        setArmSetpoint(68.5);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
    }

    public static void teleop() {
        double armCurrentAngle = getArmDegree(); // get the angular position

        // encoder reset
        if (Robot.xbox.getBackButton()) {
            armEncoder.reset();
        }

        // adjust kAP
        kAP = SmartDashboard.getNumber("arm_kP", kAP);
        armPID.setP(kAP);

        // rotate arm
        if (Robot.xbox.getXButton()) {
            setArmSetpoint(35.2);
        } else if (Robot.xbox.getYButton()) {
            setArmSetpoint(68.5);
        } else if (Robot.xbox.getLeftBumper()) {
            // control through xbox, for test
            double rotate = (Robot.xbox.getLeftTriggerAxis() - Robot.xbox.getRightTriggerAxis()) * 0.2;
            armMotor.set(rotate);
        } else {
            double armAngleModify = (Robot.xbox.getLeftTriggerAxis() -
                    Robot.xbox.getRightTriggerAxis()) * 0.01;
            setArmSetpoint(armPID.getSetpoint() + armAngleModify);
        }

        controlLoop();

        // put dashboard
        SmartDashboard.putNumber("arm_angle", armCurrentAngle);
        SmartDashboard.putNumber("arm_setpoint", armPID.getSetpoint());
        SmartDashboard.putNumber("arm_enc", armEncoder.get());
    }

    public static void controlLoop() {
        var armVolt = armPID.calculate(getArmDegree());
        if (Math.abs(armVolt) > armVoltLimit) {
            armVolt = armVoltLimit * (armVolt > 0 ? 1 : -1);
        }
        armMotor.setVoltage(armVolt);

        SmartDashboard.putNumber("arm_volt", armVolt);
    }

    // do the number of turns calculate(to a particular angle)
    public static double getArmDegree() {
        double armRate = armEncoder.get() * 360 / armEncoderPulse;
        return armRate;
    }

    public static void setArmSetpoint(double setpoint) {
        // Check if arm exceed it's physical limit
        if (setpoint < armAngleMin) {
            setpoint = armAngleMin;
        } else if (setpoint > armAngleMax) {
            setpoint = armAngleMax;
        }
        armPID.setSetpoint(setpoint);
    }

    // for NewAutoEngine
    public static int autoArmControl(int modeLine, int modeArm) {
        switch (modeArm) {
            case 0: // the beginning position
                setArmSetpoint(68.5);
                break;
            case 1: // the second level
                setArmSetpoint(-10);
                break;
            case 2: // the third level
                setArmSetpoint(35.5);
                break;
            default:
                break;
        }

        switch (modeLine) {
            case 0:// the beginning position
                Line.linePID.setSetpoint(0);
            case 0: // the beginning position
                Line.LinePID.setSetpoint(0);
                break;
            case 2: // the second level
                Line.linePID.setSetpoint(33.02);
                break;
            case 3:// the third level
                Line.linePID.setSetpoint(86.15);// the third level
            case 3: // the third level
                Line.LinePID.setSetpoint(86.15);// the third level
                break;
            default:
                break;
        }

        controlLoop();
        Line.Controlloop();
        SmartDashboard.putNumber("line enc", Line.lineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line length", Line.getEncoderTolength());
        SmartDashboard.putNumber("line_enc", Line.LineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line_length", Line.positionToLength());
        return 0;
    }
}