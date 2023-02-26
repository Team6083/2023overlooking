package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Line {
    // motor
    protected static WPI_TalonSRX LineMotor;// take up and pay off device
    private static final int line = 17;

    // pid
    private static double kLP = 0.3;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected static PIDController LinePID;
    private static double lineLengthModify = 0;

    public static void init() {
        // motor
        LineMotor = new WPI_TalonSRX(line);
        LineMotor.setInverted(true);

        // encoder
        LineMotor.setSelectedSensorPosition(0);

        // pid
        LinePID = new PIDController(kLP, kLI, kLD);
        LinePID.setSetpoint(0);

        // put dashboard
        SmartDashboard.putNumber("line_kP", kLP);
    }

    public static void teleop() {
        double length = positionToLength(); // get length position

        if (Robot.xbox.getRawButton(8)) {
            LinePID.setSetpoint(0);
        }

        // adjust P
        kLP = SmartDashboard.getNumber("line_kP", kLP);
        LinePID.setP(kLP);

        lineLengthModify = 0;
        if (Robot.xbox.getYButtonPressed()) {
            LinePID.setSetpoint(33.02);
            lineLengthModify = 0;
        // } else if (length > 122 * (1 / Math.cos(35.2)) - 58) {  //the length of the outside bumper
        //     lineLengthModify = 0.5;
        // } else if (length > 122 * Math.abs(1 / Math.cos(Arm.positionToDegree())) - 58) {
        //     lineLengthModify = -0.5;
        } else if (Robot.xbox.getPOV() == 0) {
            lineLengthModify = 0.5;
            LinePID.setSetpoint(LinePID.getSetpoint() + lineLengthModify);
        } else if (Robot.xbox.getPOV() == 180) {
            lineLengthModify = -0.5;
            LinePID.setSetpoint(LinePID.getSetpoint() + lineLengthModify);
        }

        if (length < 0) {//increase the length of arm  //from the top of the third arm to the intake
            LinePID.setSetpoint(0);
        } else if (length > 122) {
            LinePID.setSetpoint(122);// waiting for test
        }
        Controlloop();
        // if (Robot.xbox.getPOV() == 0) {
        // LineMotor.set(0.3);
        // } else if (Robot.xbox.getPOV() == 180) {
        // LineMotor.set(-0.3);
        // } else {
        // LineMotor.set(0);
        // }

       
        // put dashboard
        SmartDashboard.putNumber("line length", length);
        SmartDashboard.putNumber("Line_setpoint", LinePID.getSetpoint());
        SmartDashboard.putNumber("line enc", LineMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("length modify", lineLengthModify);
    }

    public static void Controlloop() {

        var LineVolt = LinePID.calculate(positionToLength());
        if (Math.abs(LineVolt) > 10) {
            LineVolt = LineVolt > 0 ? 10 : -10;
        }
        LineMotor.setVoltage(LineVolt);

        SmartDashboard.putNumber("LineVolt", LineVolt);
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
}
