package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Arm {
    // arm motor
    private static CANSparkMax armMotorLeft;
    private static CANSparkMax armMotorRight;
    private static MotorControllerGroup armMotor;
    private static final int armLeftCANId = 15;
    private static final int armRightCANId = 16;
    // line motor
    protected static WPI_TalonSRX lineMotor;
    private static final int line = 17;

    // arm encoder
    // private static Encoder armEncoder; // for normal encoder
    private static RelativeEncoder armEncoder;

    // private static RelativeEncoder armEncoder;
    // private static final int armEnc1Channel = 8; // for normal encoder
    // private static final int armEnc2Channel = 9; // for normal encoder

    // arm pid
    private static double kAP = 0.3;
    private static double kAI = 0.0;
    private static double kAD = 0.0;
    private static PIDController armPID;
    // // line pid
    private static double kLP = 0.35;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected static PIDController linePID;

    // arm value
    private static final double armEncoderPulse = 2048;
    private static final double armEncoderGearing = 198;
    private static final double armVoltLimit = 4;
    private static final double armAngleMin = -15;
    private static final double armAngleMax = 185;
    // line value
    private static final double modifiedLineVoltLimit = 3;
    private static final double lineLenghtMax = 140.0;
    private static final double lineLenghtMin = 40;

    public static void init() {
        // arm motor
        armMotorLeft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorLeft, armMotorRight);
        armMotorLeft.setInverted(true);

        // line motor
        lineMotor = new WPI_TalonSRX(line);
        lineMotor.setInverted(true);

        // arm encoder
        // armEncoder = new Encoder(armEnc1Channel, armEnc2Channel); // for normal
        // encoder
        // armEncoder.setReverseDirection(true);

        // encoder
        lineMotor.setSensorPhase(false);
        lineMotor.setSelectedSensorPosition(0);
        armEncoder = armMotorLeft.getEncoder(); // for sparkmax encoder

        // arm pid
        armPID = new PIDController(kAP, kAI, kAD);
        setArmSetpoint(68.5);

        // line pid
        linePID = new PIDController(kLP, kLI, kLD);

        setLineSetpoint(40);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
        SmartDashboard.putNumber("line_kP", kLP);
        SmartDashboard.putData(lineMotor);
        SmartDashboard.putData(linePID);
    }

    public static void teleop() {
        armLoop();
        lineLoop();
    }

    public static void armLoop() {
        // get the angular position
        double armCurrentAngle = getArmDegree();

        // encoder reset
        if (Robot.mainController.getBackButton()) {
            // armEncoder.reset(); // for normal encoder
            armEncoder.setPosition(0); // for sparkmax encoder
            setArmSetpoint(0);
        }

        // adjust kAP
        kAP = SmartDashboard.getNumber("arm_kP", kAP);
        armPID.setP(kAP);

        // rotate arm
        boolean armInManual = (Robot.mainController.getAButton());
        double armAngleModify = 0;
        if (Robot.viceController.getBackButton()) {
            if (Robot.viceController.getRightBumper() || Robot.viceController.getLeftBumper()) {
                setArmSetpoint(144.8);
            } else if (Robot.viceController.getPOV() == 270) {
                setArmSetpoint(111.5);
            } else if (Robot.viceController.getBButton()) {
                setArmSetpoint(151.62);
            } else {
                armAngleModify = (Robot.mainController.getLeftTriggerAxis() -
                        Robot.mainController.getRightTriggerAxis()) * -0.3;
                setArmSetpoint(armPID.getSetpoint() + armAngleModify);
            }
        } else {
            if (Robot.viceController.getRightBumper() || Robot.viceController.getLeftBumper()) {
                setArmSetpoint(35.2);
            } else if (Robot.viceController.getPOV() == 270) {
                setArmSetpoint(68.5);
            } else if (Robot.viceController.getBButton()) {
                setArmSetpoint(28.38);
            } else {
                armAngleModify = (Robot.mainController.getLeftTriggerAxis() -
                        Robot.mainController.getRightTriggerAxis()) * -0.3;
                setArmSetpoint(armPID.getSetpoint() + armAngleModify);
            }
        }
        // if (Robot.viceController.getRightBumper() ||
        // Robot.viceController.getLeftBumper()) {
        // setArmSetpoint(35.2);
        // } else if (Robot.viceController.getPOV() == 270) {
        // setArmSetpoint(68.5);
        // } else if (Robot.viceController.getBButton()) {
        // setArmSetpoint(28.38);
        // }else {
        // armAngleModify = (Robot.mainController.getLeftTriggerAxis() -
        // Robot.mainController.getRightTriggerAxis()) * -0.3;
        // setArmSetpoint(armPID.getSetpoint() + armAngleModify);
        // }
        if (armInManual) {
            // control through xbox, for test
            double rotate = (Robot.mainController.getLeftTriggerAxis() -
                    Robot.mainController.getRightTriggerAxis()) * -0.15;
            armMotor.set(rotate);
            setArmSetpoint(armCurrentAngle);
        } else {
            armControlLoop();
        }
        // put dashboard
        SmartDashboard.putNumber("arm_setpoint", armPID.getSetpoint());
        SmartDashboard.putNumber("arm_current_angle", armCurrentAngle);
        SmartDashboard.putNumber("arm_angle_modify", armAngleModify);
        // double limit = Math.abs(74 / 253125 * Math.pow(armCurrentAngle, 3)
        // + 11 / 1000 *Math.pow(armCurrentAngle, 2) + 323 / 9000 * armCurrentAngle +
        // 69);
        // System.out.println("first part:"+74 / 253125 * Math.pow(armCurrentAngle, 3));
        // System.out.println("second part:"+11 / 1000 *Math.pow(armCurrentAngle, 2));
        // System.out.println("thrid: "+323 / 9000 * armCurrentAngle);
        // SmartDashboard.putNumber("delta_long_function", limit);

    }

    public static void lineLoop() {
        // get length position
        final double lineCurrentLength = getEncoderToLength();

        // encoder reset
        if (Robot.mainController.getStartButton()) {
            lineMotor.setSelectedSensorPosition(40);
            setLineSetpoint(40);
        }
        // adjust kLP
        kLP = SmartDashboard.getNumber("line_kP", kLP);
        linePID.setP(kLP);

        // stretch line
        boolean lineInManual = (Robot.mainController.getXButton());
        double lineLengthModify = 0.0;
        if (Robot.viceController.getLeftBumper()) {
            setLineSetpoint(73.02);
        } else if (Robot.viceController.getRightBumper()) {
            setLineSetpoint(126.15);
        } else if (Robot.viceController.getBButton()) {
            setLineSetpoint(98.14);
        } else if (Robot.mainController.getPOV() == 0) {
            lineLengthModify = 0.3;
            setLineSetpoint(linePID.getSetpoint() + lineLengthModify);
        } else if (Robot.mainController.getPOV() == 180) {
            lineLengthModify = -0.4;
            setLineSetpoint(linePID.getSetpoint() + lineLengthModify);
        }

        final double lineMotorCurrentLimit = 10;
        double lineMotorCurrent = getLineCurrent();
        if (lineMotorCurrent > lineMotorCurrentLimit) {
            lineMotor.stopMotor();
        } else {
            if (lineInManual) {
                if (Robot.mainController.getPOV() == 0) {
                    lineMotor.set(0.3);
                } else if (Robot.mainController.getPOV() == 180) {
                    lineMotor.set(-0.3);
                } else {
                    lineMotor.set(0);
                }
                setLineSetpoint(lineCurrentLength);
            } else {
                lineControlLoop();
            }
        }

        // put dashboard
        SmartDashboard.putNumber("line_setpoint", linePID.getSetpoint());
        SmartDashboard.putNumber("line_current_length", lineCurrentLength);
        SmartDashboard.putNumber("line_length_modify", lineLengthModify);
        SmartDashboard.putNumber("line_current", lineMotorCurrent);
    }

    public static void armControlLoop() {

        var armVolt = armPID.calculate(getArmDegree());

        double modifiedArmVolt = armVolt;
        if (Math.abs(modifiedArmVolt) > armVoltLimit) {
            modifiedArmVolt = armVoltLimit * (armVolt > 0 ? 1 : -1);
        }

        armMotor.setVoltage(modifiedArmVolt);
        setLineSetpoint(linePID.getSetpoint());
        lineControlLoop();

        SmartDashboard.putNumber("arm_orig_volt", armVolt);
        SmartDashboard.putNumber("arm_volt", modifiedArmVolt);
    }

    public static void lineControlLoop() {

        var lineVolt = linePID.calculate(getEncoderToLength());
        double modifiedLineVolt = lineVolt;
        if (Math.abs(lineVolt) > modifiedLineVoltLimit) {
            modifiedLineVolt = modifiedLineVoltLimit * (modifiedLineVolt > 0 ? 1 : -1);
        }
        lineMotor.setVoltage(modifiedLineVolt);

        SmartDashboard.putNumber("line_orig_volt", lineVolt);
        SmartDashboard.putNumber("line_volt", modifiedLineVolt);
    }

    // do the number of turns calculate(to a particular angle)
    public static double getArmDegree() {
        // double armRate = armEncoder.get() * 360 / armEncoderPulse; // for normal
        // encoder
        // SmartDashboard.putNumber("arm_encoder", armEncoder.get()); // for normal
        // encoder
        double armRate = armEncoder.getPosition() * 360 / armEncoderGearing; // for
        // sparkmax encoder
        SmartDashboard.putNumber("arm_encoder", armEncoder.getPosition()); // for
        // sparkmax encoder

        SmartDashboard.putNumber("arm_angle", armRate);
        return armRate;
    }

    // do the number of turns calculate(to a particular length)
    public static double getEncoderToLength() {
        double x = lineMotor.getSelectedSensorPosition();
        double length = 0.00473 * x - 0.0000000348 * x * x + 40;
        SmartDashboard.putNumber("line_encoder", lineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line_length", length);
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

    public static void setLineSetpoint(double setpoint) {
        // Check if line exceed it's physical limit
        // double angle = getArmDegree();
        if (setpoint < lineLenghtMin) {
            setpoint = lineLenghtMin;
        } else if (setpoint > lineLenghtMax) {
            setpoint = lineLenghtMax;
        }
        // check if line exceed it's game limit
        double radian = Math.toRadians(getArmDegree());
        if (setpoint > 175 * Math.abs(1 / Math.cos(radian)) - 60) {
            setpoint = 175 * Math.abs(1 / Math.cos(getArmDegree()));
        }
        linePID.setSetpoint(setpoint);

        SmartDashboard.putNumber("rafdian", radian);
        SmartDashboard.putNumber("delta_long_cos", 175 * Math.abs(1 / Math.cos(radian)) - 60);

    }

    public static double getLineCurrent() {
        return Robot.pd.getCurrent(0);
    }
}