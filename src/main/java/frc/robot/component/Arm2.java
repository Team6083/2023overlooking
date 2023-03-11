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

public class Arm2 {
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
    private static double[][] armAngleSetpoints = {{35.6, 28.38, 68.5}, {130, 151, 101.5}};
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
        lineMotor.setSensorPhase(true);

        // arm encoder
        // armEncoder = new Encoder(armEnc1Channel, armEnc2Channel); // for normal
        // encoder
        // armEncoder.setReverseDirection(true);

        // encoder
        armEncoder = armMotorLeft.getEncoder(); // for sparkmax encoder 
        lineMotor.setSelectedSensorPosition(0);
        
        // arm pid
        armPID = new PIDController(kAP, kAI, kAD);
        setArmSetpoint(68.5);

        // line pid
        linePID = new PIDController(kLP, kLI, kLD);

        setLineSetpoint(40);

        // put dashboard
        SmartDashboard.putNumber("arm_kP", kAP);
        SmartDashboard.putNumber("line_kP", kLP);
        SmartDashboard.putData("line_motor", lineMotor);
        SmartDashboard.putData("line_PID", linePID);
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
        int backButtonPressed = Robot.viceController.getBackButton() ? 1 : 0;
        
        if (Robot.viceController.getRightBumper() || Robot.viceController.getLeftBumper()) {
            setArmSetpoint(armAngleSetpoints[backButtonPressed][0]);
        } else if (Robot.viceController.getPOV() == 270) {
            setArmSetpoint(armAngleSetpoints[backButtonPressed][1]);
        } else if (Robot.viceController.getBButton()) {
            setArmSetpoint(armAngleSetpoints[backButtonPressed][2]);
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
        SmartDashboard.putNumber("arm_current_angle", armCurrentAngle);
        SmartDashboard.putNumber("arm_angle_modify", armAngleModify);

    }

    public static void lineLoop() {
        // get length position
        final double lineCurrentLength = getEncoderToLength();

        // encoder reset
        if (Robot.mainController.getStartButton()) {
            lineMotor.setSelectedSensorPosition(0);
            setLineSetpoint(40);
        }
        // adjust kLP
        kLP = SmartDashboard.getNumber("line_kP", kLP);
        linePID.setP(kLP);

        // stretch line
        boolean lineInManual = (Robot.mainController.getXButton());
        double lineLengthModify = 0.0;
        if (Robot.viceController.getLeftBumper()) {
            setLineSetpoint(84.6);
        } else if (Robot.viceController.getRightBumper()) {
            setLineSetpoint(131);
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
        double cal1 = 0.00473 * x;
        double cal2 = 0.0000000348 * x * x;
        double length = cal1-cal2+40;
        SmartDashboard.putNumber("line_encoder", lineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("line_length", length);
        SmartDashboard.putNumber("X", x);
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
        if (setpoint > 170 * Math.abs(1 / Math.cos(radian)) - 60) {
            setpoint = 170 * Math.abs(1 / Math.cos(radian))-60;
        }
        linePID.setSetpoint(setpoint);

        SmartDashboard.putNumber("rafdian", radian);
        SmartDashboard.putNumber("delta_long_cos", 170 * Math.abs(1 / Math.cos(radian)) - 60);

    }

    public static double getLineCurrent() {
        return Robot.pd.getCurrent(0);
    }
}