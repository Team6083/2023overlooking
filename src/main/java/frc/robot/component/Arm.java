package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.hardware.armEncoder.ArmEncoder;
import frc.robot.hardware.armEncoder.NeoArmEncoder;
import frc.robot.hardware.armEncoder.RevHexArmEncoder;

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

    // arm encoder
    private static ArmEncoder armEncoder;
    private static final boolean useNEOEnc = false;

    // arm pid
    private static double kAP = 0.3;
    private static double kAI = 0.0;
    private static double kAD = 0.0;
    private static PIDController armPID;
    // line pid
    private static double kLP = 0.35;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected static PIDController linePID;

    // arm value
    private static final double armVoltLimit = 4;
    private static final double armAngleMin = -10;
    private static final double armAngleMax = 185;
    // line value
    private static final double lineVoltLimit = 3;
    private static final double lineLenghtMax = 100.0;
    private static final double lineLenghtMin = 0;

    public static void init() {
        // arm motor
        armMotorleft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorright = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorleft, armMotorright);
        armMotorleft.setInverted(true);

        // line motor
        lineMotor = new WPI_TalonSRX(line);
        lineMotor.setInverted(true);

        // arm encoder
        if (useNEOEnc) {
            armEncoder = new NeoArmEncoder(armMotorleft.getEncoder());
        } else {
            armEncoder = new RevHexArmEncoder(8, 9);
        }

        // arm pid
        armPID = new PIDController(kAP, kAI, kAD);
        setArmSetpoint(68.5);

        // line pid
        linePID = new PIDController(kLP, kLI, kLD);

        setLineSetpoint(0);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
        SmartDashboard.putNumber("line_kP", kLP);
    }

    public static void teleop() {
        armLoop();
        lineLoop();
    }

    public static void armLoop() {
        // get the angular position
        double armCurrentAngle = getArmDegree();

        // encoder reset
        if (Robot.viceController.getBackButton()) {
            armEncoder.reset();
        }

        // adjust kAP
        kAP = SmartDashboard.getNumber("arm_kP", kAP);
        armPID.setP(kAP);

        // rotate arm
        boolean armInManual = (Robot.mainController.getXButton());
        double armAngleModify = 0;
        if (Robot.viceController.getRightBumper() || Robot.viceController.getLeftBumper()) {
            setArmSetpoint(35.2);
        } else if (Robot.viceController.getPOV() == 270) {
            setArmSetpoint(68.5);
        } else if (Robot.viceController.getBButton()) {
            setArmSetpoint(28.38);
        } else if (Robot.viceController.getYButton()) {
            setArmSetpoint(90);
        } else {
            armAngleModify = (Robot.mainController.getLeftTriggerAxis() -
                    Robot.mainController.getRightTriggerAxis()) * -0.3;
            setArmSetpoint(armPID.getSetpoint() + armAngleModify);
        }
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
        SmartDashboard.putNumber("aem_current_angle", armCurrentAngle);
        SmartDashboard.putNumber("arm_angle_modify", armAngleModify);
    }

    public static void lineLoop() {
        // get length position
        final double lineCurrentLength = getEncoderToLength();

        // encoder reset
        if (Robot.viceController.getStartButton()) {
            lineMotor.setSelectedSensorPosition(0);
            setLineSetpoint(0);
        }
        // adjust kLP
        kLP = SmartDashboard.getNumber("line_kP", kLP);
        linePID.setP(kLP);

        // stretch line
        boolean lineInManual = (Robot.mainController.getXButton());
        double lineLengthModify = 0.0;
        if (Robot.viceController.getLeftBumper()) {
            setLineSetpoint(33.02);
        } else if (Robot.viceController.getRightBumper()) {
            setLineSetpoint(86.15);
        } else if (Robot.viceController.getBButton()) {
            setArmSetpoint(28.38);
        } else if (Robot.viceController.getYButton()) {
            setLineSetpoint(58.14);
        } else if (Robot.mainController.getPOV() == 0) {
            lineLengthModify = 0.3;
            setLineSetpoint(linePID.getSetpoint() + lineLengthModify);
        } else if (Robot.mainController.getPOV() == 180) {
            lineLengthModify = -0.4;
            setLineSetpoint(linePID.getSetpoint() + lineLengthModify);
        }
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
        // put dashboard
        SmartDashboard.putNumber("line_setpoint", linePID.getSetpoint());
        SmartDashboard.putNumber("line_current_length", lineCurrentLength);
        SmartDashboard.putNumber("line_length_modify", lineLengthModify);

    }

    public static void armControlLoop() {

        var armVolt = armPID.calculate(getArmDegree());

        double modifiedArmVolt = armVolt;
        if (Math.abs(modifiedArmVolt) > armVoltLimit) {
            modifiedArmVolt = armVoltLimit * (armVolt > 0 ? 1 : -1);
        }
        armMotor.setVoltage(modifiedArmVolt);

        SmartDashboard.putNumber("arm_orig_volt", armVolt);
        SmartDashboard.putNumber("arm_volt", modifiedArmVolt);
    }

    public static void lineControlLoop() {

        var lineVolt = linePID.calculate(getEncoderToLength());
        if (Math.abs(lineVolt) > 10) {
            lineVolt = lineVoltLimit * (lineVolt > 0 ? 1 : -1);
        }
        lineMotor.setVoltage(lineVolt);

        SmartDashboard.putNumber("line_volt", lineVolt);
    }

    // do the number of turns calculate(to a particular angle)
    public static double getArmDegree() {
        var angle = armEncoder.getAngleDegree();

        SmartDashboard.putNumber("arm_encoder", armEncoder.getRawPosition());
        SmartDashboard.putNumber("arm_angle", angle);
        return angle;
    }

    // do the number of turns calculate(to a particular length)
    public static double getEncoderToLength() {
        if (lineMotor.getSelectedSensorPosition() < 0) {
            lineMotor.setSelectedSensorPosition(0.0);
        }
        double x = lineMotor.getSelectedSensorPosition();
        double length = 0.00473 * x - 0.0000000348 * x * x;
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
        if (setpoint < lineLenghtMin) {
            setpoint = lineLenghtMin;
        } else if (setpoint > lineLenghtMax) {
            setpoint = lineLenghtMax;
        }
        // check if line exceed it's game limit
        if (setpoint > 175 * Math.abs(1 / Math.cos(getArmDegree())) - 58) {
            setpoint = 175 * Math.abs(1 / Math.cos(getArmDegree())) - 58;
        }
        linePID.setSetpoint(setpoint);
    }

}