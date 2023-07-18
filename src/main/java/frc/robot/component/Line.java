package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Line {
    // line motor
    private static final int lineId = 17;
    private static WPI_TalonSRX lineMotor;

    // PID controller for line motor
    private static double kLP = 1.2;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected PIDController linePID;

    private static double lineLengthOffset;

    // constants for line motor
    private static final double modifiedLineVoltPLimit = 3;
    private static final double modifiedLineVoltNLimit = -5;
    private static final double maxLineLengthLimit = 90;
    private static final double minLineLengthLimit = 40;

    public Line(double lineInitLength) {
        lineMotor = new WPI_TalonSRX(lineId);
        lineMotor.setInverted(true);
        lineMotor.setSensorPhase(true);
        resetEncoder();

        linePID = new PIDController(kLP, kLI, kLD);
        linePID.setSetpoint(lineInitLength);

        lineLengthOffset = lineInitLength;
        SmartDashboard.putData("line_PID", linePID);
        SmartDashboard.putData("line_motor", lineMotor);
    }

    public void manualControlLoop(double manualControlSpeed) {
        lineMotor.set(manualControlSpeed);
        linePID.setSetpoint(getLineLength());
    }

    public void pidControlLoop() {
        double lineVolt = linePID.calculate(getLineLength());
        if (lineVolt > modifiedLineVoltPLimit) {
            lineVolt = modifiedLineVoltPLimit;
        } else if (lineVolt < modifiedLineVoltNLimit) {
            lineVolt = modifiedLineVoltNLimit;
        }
        lineMotor.setVoltage(lineVolt);

        SmartDashboard.putNumber("line_volt", lineVolt);
    }

    public double getPIDSetpoint() {
        return linePID.getSetpoint();
    }

    public void setPIDSetpoint(double setpoint) {
        final double currentSetpoint = linePID.getSetpoint();
        if (isPhyLimitExceed(currentSetpoint) != 0) {
            return;
        }
        if (isPhyLimitExceed(setpoint) == -1) {
            setpoint = minLineLengthLimit;
        } else if (isPhyLimitExceed(setpoint) == 1) {
            setpoint = maxLineLengthLimit;
        }
        linePID.setSetpoint(setpoint);
    }

    public void resetEncoder() {
        lineLengthOffset = 0;
        lineMotor.setSelectedSensorPosition(0);
    }

    public void resetSetpoint() {
        linePID.setSetpoint(40);
    }

    public void stopMotor() {
        lineMotor.stopMotor();
    }

    public double getLineLength() {
        double x = lineMotor.getSelectedSensorPosition();
        double cal1 = 0.00473 * x;
        double cal2 = 0.0000000348 * x * x;
        double length = cal1 - cal2;
        return length + lineLengthOffset;
    }

    private int isPhyLimitExceed(double angle) {
        return angle < minLineLengthLimit ? -1 : (angle > maxLineLengthLimit ? 1 : 0);
    }
}
