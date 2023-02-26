package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Line {
    // motor
    protected static WPI_TalonSRX lineMotor;
    private static final int line = 17;

    // line pid
    private static double kLP = 0.3;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected static PIDController linePID;

    // value
    private static final double lineLenghtMax = 122.0;
    private static final double lineLenghtMin = 0.0;

    public static void init() {
        // motor
        lineMotor = new WPI_TalonSRX(line);
        lineMotor.setInverted(true);

        // encoder
        lineMotor.setSelectedSensorPosition(0);

        // pid
        linePID = new PIDController(kLP, kLI, kLD);
        linePID.setSetpoint(0);

        // put dashboard
        SmartDashboard.putNumber("line_kP", kLP);
    }

    public static void teleop() {
        double length = getEncoderToLength(); // get length position

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
        controlLoop();
        // if (Robot.xbox.getPOV() == 0) {
        // LineMotor.set(0.3);
        // } else if (Robot.xbox.getPOV() == 180) {
        // LineMotor.set(-0.3);
        // } else {
        // LineMotor.set(0);
        // }

        // put dashboard
        SmartDashboard.putNumber("line_length", length);
        SmartDashboard.putNumber("line_setpoint", linePID.getSetpoint());
        SmartDashboard.putNumber("line_enc", lineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("length_modify", lineLengthModify);
    }

    public static void controlLoop() {

        var lineVolt = linePID.calculate(getEncoderToLength());
        if (Math.abs(lineVolt) > 10) {
            lineVolt = lineVolt > 0 ? 10 : -10;
        }
        lineMotor.setVoltage(lineVolt);

        SmartDashboard.putNumber("line_Volt", lineVolt);
    }

    // do the number of turns calculate(to a particular length)
    public static double getEncoderToLength() {
        if (lineMotor.getSelectedSensorPosition() < 0) {
            lineMotor.setSelectedSensorPosition(0.0);
        }
        double x = lineMotor.getSelectedSensorPosition();
        double length = 0.00473 * x - 0.0000000348 * x * x;
        return length;
    }

    public static void setLineSetpoint() {
        // Check if line exceed it's physical limit
        double length = getEncoderToLength();
        if (length < lineLenghtMin) {
            length = lineLenghtMin;
        } else if (length > lineLenghtMax) {
            length = lineLenghtMax;
        }

        linePID.setSetpoint(length);
    }
}
