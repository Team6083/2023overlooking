package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Arm {
    // motor
    private static CANSparkMax ArmMotorleft;// rotate arm
    private static CANSparkMax ArmMotorright;
    private static MotorControllerGroup ArmMotor;
    private static WPI_TalonSRX LineMotor;// take up and pay off device
    private static final int armL = 15;
    private static final int armR = 16;
    private static final int line = 17;

    // encoder
    private static RelativeEncoder ArmEncoder;

    // pid
    private static double kAP = 0.35;
    private static double kAI = 0.0;
    private static double kAD = 0.0;
    private static PIDController ArmPID;
    private static double rotate;
    private static double armAngleModify = 0;
    private static double kLP = 0.3;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    private static PIDController LinePID;
    private static double lineLengthModify = 0;

    // value
    private static final double ArmencoderPulse = 42;// do the number of turns calculate
    private static final double Armgearing = 198;

    public static void init() {
        // motor
        ArmMotorleft = new CANSparkMax(armL, MotorType.kBrushless);
        ArmMotorright = new CANSparkMax(armR, MotorType.kBrushless);
        ArmMotor = new MotorControllerGroup(ArmMotorleft, ArmMotorright);
        ArmMotorleft.setInverted(true);
        LineMotor = new WPI_TalonSRX(line);
        LineMotor.setInverted(true);

        // encoder
        ArmEncoder = ArmMotorleft.getEncoder();
        LineMotor.configClearPositionOnQuadIdx(false, 10);
        LineMotor.setSelectedSensorPosition(0);

        // pid
        ArmPID = new PIDController(kAP, kAI, kAD);
        LinePID = new PIDController(kLP, kLI, kLD);
        // ArmPID.setSetpoint(68.5);
        LinePID.setSetpoint(0);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
        SmartDashboard.putNumber("line_kP", kLP);
    }

    public static void teleop() {
        double angle = positionToDegree(); // get the angular position
        // double length = positionToLength(); // get length position

        // encoder reset
        if (Robot.xbox.getRawButton(7)) {
            ArmMotorleft.getEncoder().setPosition(0);
            ArmMotorright.getEncoder().setPosition(0);
            ArmEncoder = ArmMotorleft.getEncoder();
        }

        // adjust P
        kAP = SmartDashboard.getNumber("arm_kP", kAP);
        ArmPID.setP(kAP);
        kLP = SmartDashboard.getNumber("line_kP", kLP);
        LinePID.setP(kLP);

        // rotate arm
        // if (Robot.xbox.getXButton()) {
        // ArmPID.setSetpoint(35.2);
        // } else if (Robot.xbox.getYButton()) {
        // ArmPID.setSetpoint(68.5);
        // } else {
        // armAngleModify = (Robot.xbox.getLeftTriggerAxis() -
        // Robot.xbox.getRightTriggerAxis()) * 0.01;
        // ArmPID.setSetpoint(ArmPID.getSetpoint() + armAngleModify);
        // }
        rotate = (Robot.xbox.getLeftTriggerAxis() - Robot.xbox.getRightTriggerAxis()) * 0.2;
        ArmMotor.set(rotate);

        // take up and pay off device
        // if (Robot.xbox.getYButtonPressed()) {
        // // LinePID.setSetpoint(33.02);
        // lineLengthModify = 0;
        // } else if (length > 122 * (1 / Math.cos(35.2)) - 58) {
        // lineLengthModify -= 0.5;
        // } else if (length > 122 * Math.abs(1 / Math.cos(angle)) - 58) {
        // lineLengthModify -= 0.5;
        // } else if (Robot.xbox.getPOV() == 0) {
        // lineLengthModify += 0.5;
        // } else if (Robot.xbox.getPOV() == 180) {
        // lineLengthModify -= 0.5;
        // } else {
        // LineMotor.set(0);
        // }
        // LinePID.setSetpoint(LinePID.getSetpoint() + lineLengthModify);

        Controlloop();

        // put dashboard
        SmartDashboard.putNumber("arm angle", angle);
        // SmartDashboard.putNumber("line length", length);
        SmartDashboard.putNumber("Arm_setpoint", ArmPID.getSetpoint());
        SmartDashboard.putNumber("Line_set", LinePID.getSetpoint());
        SmartDashboard.putNumber("arm enc", ArmEncoder.getPosition());
        SmartDashboard.putNumber("line enc", LineMotor.getSelectedSensorPosition());
    }

    public static void Controlloop() {
        // var ArmVolt = ArmPID.calculate(positionToDegree());
        // if (Math.abs(ArmVolt) > 10) {
        // ArmVolt = 10 * (ArmVolt > 0 ? 1 : -1);
        // }
        // ArmMotor.setVoltage(ArmVolt);

        // var LineVolt = LinePID.calculate(positionToLength());
        // if (Math.abs(LineVolt) > 10) {
        // LineVolt = LineVolt > 0 ? 10 : -10;
        // }
        // LineMotor.setVoltage(LineVolt);

        // SmartDashboard.putNumber("ArmVolt", ArmVolt);
        // SmartDashboard.putNumber("LineVolt", LineVolt);
    }

    // do the number of turns calculate(to a particular angle)
    public static double positionToDegree() {
        double armRate = ArmEncoder.getPosition() * 360 / (Armgearing *
                ArmencoderPulse);
        return armRate;
    }

    // do the number of turns calculate(to a particular length)
    public static double positionToLength() {
        if (LineMotor.getSelectedSensorPosition() < 0) {
            LineMotor.setSelectedSensorPosition(0.0);
        }
        double x = LineMotor.getSelectedSensorPosition();
        double length = 0.00473 * x - 0.0000000348 * x * x;
        return length;
    }

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
                LinePID.setSetpoint(0);
                break;
            case 2: // the second level
                LinePID.setSetpoint(33.02);
                break;
            case 3:// the third level
                LinePID.setSetpoint(86.15);// the third level
                break;
            default:
                break;
        }

        Controlloop();

        SmartDashboard.putNumber("line enc", LineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line length", positionToLength());
        return 0;
    }
}