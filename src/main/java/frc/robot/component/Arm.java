package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Arm {
    // arm motor
    private static CANSparkMax armMotorleft;
    private static CANSparkMax armMotorright;
    private static MotorControllerGroup armMotor;
    private static final int armLeftCANId = 15;
    private static final int armRightCANId = 16;
    // line motor
    protected static WPI_TalonSRX lineMotor;
    private static final int line = 17;

    // encoder
    private static Encoder armEncoder;
    private static final int armEnc1Channel = 8;
    private static final int armEnc2Channel = 9;

    // arm pid
    private static double kAP = 0.35;
    private static double kAI = 0.0;
    private static double kAD = 0.0;
    private static PIDController armPID;
    // line pid
    private static double kLP = 0.3;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected static PIDController linePID;

    // arm value
    private static final double armEncoderPulse = 2048;
    private static final double armVoltLimit = 10;
    private static final double armAngleMin = -30;
    private static final double armAngleMax = 210;
    // line value
    private static final double lineLenghtMax = 122.0;
    private static final double lineLenghtMin = 0.0;

    public static void init() {
        // arm motor
        armMotorleft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorright = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorleft, armMotorright);
        armMotorleft.setInverted(true);
        //line motor
        lineMotor = new WPI_TalonSRX(line);
        lineMotor.setInverted(true);

        // encoder
        armEncoder = new Encoder(armEnc1Channel, armEnc2Channel);
        lineMotor.setSelectedSensorPosition(0);

        // arm pid
        armPID = new PIDController(kAP, kAI, kAD);
        setArmSetpoint(68.5);
        // line pid
        linePID = new PIDController(kLP, kLI, kLD);
        linePID.setSetpoint(0);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
        SmartDashboard.putNumber("line_kP", kLP);
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

        armcontrolLoop();


        double length = getEncoderTolength(); // get length position

        if (Robot.xbox.getRawButton(8)) {
            linePID.setSetpoint(0);
        }

        // adjust P
        kLP = SmartDashboard.getNumber("line_kP", kLP);
        linePID.setP(kLP);

        Double lineLengthModify = 0.0;
        if (Robot.xbox.getYButtonPressed()) {
            linePID.setSetpoint(33.02);
            lineLengthModify = 0.0;
            // } else if (length > 122 * (1 / Math.cos(35.2)) - 58) { //the length of the
            // outside bumper
            // lineLengthModify = 0.5;
            // } else if (length > 122 * Math.abs(1 / Math.cos(Arm.positionToDegree())) -
            // 58) {
            // lineLengthModify = -0.5;
        } else if (Robot.xbox.getPOV() == 0) {
            lineLengthModify = 0.5;
            linePID.setSetpoint(linePID.getSetpoint() + lineLengthModify);
        } else if (Robot.xbox.getPOV() == 180) {
            lineLengthModify = -0.5;
            linePID.setSetpoint(linePID.getSetpoint() + lineLengthModify);
        }

        setLineSetpoint();
        linecontrolloop();
        // if (Robot.xbox.getPOV() == 0) {
        // LineMotor.set(0.3);
        // } else if (Robot.xbox.getPOV() == 180) {
        // LineMotor.set(-0.3);
        // } else {
        // LineMotor.set(0);
        // }

        // put dashboard
        SmartDashboard.putNumber("arm_angle", armCurrentAngle);
        SmartDashboard.putNumber("arm_setpoint", armPID.getSetpoint());
        SmartDashboard.putNumber("arm_enc", armEncoder.get());
        SmartDashboard.putNumber("line_length", length);
        SmartDashboard.putNumber("line_setpoint", linePID.getSetpoint());
        SmartDashboard.putNumber("line_enc", lineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("length_modify", lineLengthModify);
    }

    public static void armcontrolLoop() {
        var armVolt = armPID.calculate(getArmDegree());
        if (Math.abs(armVolt) > armVoltLimit) {
            armVolt = armVoltLimit * (armVolt > 0 ? 1 : -1);
        }
        armMotor.setVoltage(armVolt);

        SmartDashboard.putNumber("arm_volt", armVolt);
    }

    public static void linecontrolloop() {

        var lineVolt = linePID.calculate(getEncoderTolength());
        if (Math.abs(lineVolt) > 10) {
            lineVolt = lineVolt > 0 ? 10 : -10;
        }
        lineMotor.setVoltage(lineVolt);

        SmartDashboard.putNumber("line_Volt", lineVolt);
    }

    // do the number of turns calculate(to a particular angle)
    public static double getArmDegree() {
        double armRate = armEncoder.get() * 360 / armEncoderPulse;
        return armRate;
    }

    // do the number of turns calculate(to a particular length)
    public static double getEncoderTolength() {
        if (lineMotor.getSelectedSensorPosition() < 0) {
            lineMotor.setSelectedSensorPosition(0.0);
        }
        double x = lineMotor.getSelectedSensorPosition();
        double length = 0.00473 * x - 0.0000000348 * x * x;
        return length;
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

    public static void setLineSetpoint() {
        // Check if line exceed it's physical limit
        double lenght = getEncoderTolength();
        if (lenght < lineLenghtMin) {
            lenght = lineLenghtMin;
        } else if (lenght > lineLenghtMax) {
            lenght = lineLenghtMax;
        }

        linePID.setSetpoint(lenght);
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
                break;
            case 2: // the second level
                Line.linePID.setSetpoint(33.02);
                break;
            case 3:// the third level
                Line.linePID.setSetpoint(86.15);// the third level
                break;
            default:
                break;
        }

        armcontrolLoop();
        Line.controlloop();
        SmartDashboard.putNumber("line enc", Line.lineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line length", Line.getEncoderTolength());
        return 0;
    }
}