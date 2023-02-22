package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.lang.Math;

public class Arm {
    private static final double ArmencoderPulse = 42;// do the number of turns calculate
    private static final double Armgearing = 198;
    private static final double lineencoderPulse = 8192;
    private static final double linegearing = 64;
    private static CANSparkMax ArmMotorleft;// rotate arm
    private static CANSparkMax ArmMotorright;
    private static MotorControllerGroup Arm;
    private static WPI_TalonSRX lineMotor;// take up and pay off device
    private static final int armL = 15;
    private static final int armR = 16;
    private static final int line = 17;
    private static RelativeEncoder ArmEncoder;
    private static Double kP = 0.35;
    private static Double kI = 0.0;
    private static Double kD = 0.0;
    private static PIDController ArmPID;

    private static Double lP = 0.3;
    private static Double lI = 0.0;
    private static Double lD = 0.0;
    private static PIDController LinePID;
    private static Double rotate;
    // private static Double rotateReverse;

    public static void init() {
        ArmMotorleft = new CANSparkMax(armL, MotorType.kBrushless);
        ArmMotorright = new CANSparkMax(armR, MotorType.kBrushless);
        Arm = new MotorControllerGroup(ArmMotorleft, ArmMotorright);
        ArmMotorleft.setInverted(true);

        // ArmEncoder = ArmMotorleft.getEncoder();

        lineMotor = new WPI_TalonSRX(line);
        LinePID = new PIDController(lP, lI, lD);
        // ArmPID = new PIDController(kP, kI, kD);

        lineMotor.configClearPositionOnQuadIdx(true, 10);
        lineMotor.setInverted(true);
        // ArmMotorleft.getEncoder().setPosition(0);
        // ArmMotorright.getEncoder().setPosition(0);

        // SmartDashboard.putNumber("arm_kP", kP);
    }

    public static void teleop() {
        // rotate = (Robot.xbox.getLeftTriggerAxis() - Robot.xbox.getRightTriggerAxis())
        // * 0.2;
        // ArmMotorleft.set(rotate);
        // ArmMotorright.set(rotate);

        // kP = SmartDashboard.getNumber("arm_kP", kP);
        // ArmPID.setP(kP);
        lP = SmartDashboard.getNumber("line_kP", lP);
        lineMotor.configClearPositionOnQuadIdx(false, 10);

        // double angle = positionToDegree();// get the angular position
        // double length = positionToLength(); // get length position

        // take up and pay off device
        if (Robot.xbox.getPOV() == 0) {
            lineMotor.set(0.3);
        } else if (Robot.xbox.getPOV() == 180) {
            lineMotor.set(-0.3);
        } else {
            lineMotor.set(0);
        }
        // else if (Robot.xbox.getXButton()) {
        // if (length > 122 * (1 / Math.cos(35.2)) - 58) {
        // lineMotor.set(-0.5);
        // }
        // } else if (length > 122 * (1 / Math.cos(angle)) - 58) {
        // lineMotor.set(-0.5);
        // }
        // else {
        // lineMotor.set(0);
        // }

        if (lineMotor.getSelectedSensorPosition() < 0) {
            lineMotor.configClearPositionOnQuadIdx(true, 10);
        } else {
            lineMotor.configClearPositionOnQuadIdx(false, 10);
        }

        // if (Robot.xbox.getXButton()) {
        // ArmPID.setSetpoint(35.2);
        // } else if (Robot.xbox.getYButton()) {
        // ArmPID.setSetpoint(68.5);
        // length = 0;
        // } else {
        // double armAngleModify = (Robot.xbox.getLeftTriggerAxis() -
        // Robot.xbox.getRightTriggerAxis()) * 0.01;
        // ArmPID.setSetpoint(ArmPID.getSetpoint() + armAngleModify);
        // }
        // SmartDashboard.putNumber("setpoint", ArmPID.getSetpoint());
        // SmartDashboard.putNumber("current", positionToDegree());
        // SmartDashboard.putNumber("arm enc", ArmEncoder.getPosition());
        // SmartDashboard.putNumber("angle",angle);
        SmartDashboard.putNumber("line enc", lineMotor.getSelectedSensorPosition());
        // SmartDashboard.putNumber("length",length);
        SmartDashboard.putNumber("line length", positionToLength());
        // controlloop();
    }

    public static void Controlloop() {
        // var ArmVolt = ArmPID.calculate(positionToDegree());

        // if (Math.abs(ArmVolt) > 10) {
        // ArmVolt = 10 * (ArmVolt > 0 ? 1 : -1);
        // }
        // Arm.setVoltage(ArmVolt);

        // SmartDashboard.putNumber("ArmVolt", ArmVolt);

        var LineVolt = LinePID.calculate(positionToLength());
        if (Math.abs(LineVolt) > 10) {
            LineVolt = LineVolt > 0 ? 10 : -10;
        }
        lineMotor.setVoltage(LineVolt);
        SmartDashboard.putNumber("LineVolt", LineVolt);
    }

    // do the number of turns calculate(to a particular angle)
    // public static double positionToDegree() {
    // double armRate = ArmEncoder.getPosition() * 360 / (Armgearing *
    // ArmencoderPulse);
    // return armRate;
    // }

    // do the number of turns calculate(to a particular length)
    public static double positionToLength() {
        double x = lineMotor.getSelectedSensorPosition();
        double length = 98.4 + 0.00473 * x - 0.0000000348 * x * x;
        return length;
    }

    // public static double autoArm(double speed) {
    // Arm.set(speed);
    // return 0;
    // }

    // public static double autoLine(double speed) {
    // lineMotor.set(speed);
    // return 0;
    // }
}