package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Line {
    protected final int lineCANId = 17;

    protected WPI_TalonSRX lineMotor;

    // PID
    protected double kP = 0.35;
    protected PIDController linePID;

    protected static final double lineVoltLimit = 3;
    protected static final double lineLenghtMax = 100.0;
    protected static final double lineLenghtMin = 0;

    public Line(double initSetpoint) {
        lineMotor = new WPI_TalonSRX(lineCANId);
        lineMotor.setInverted(true);

        linePID = new PIDController(kP, 0, 0);
        linePID.setSetpoint(initSetpoint);

        SmartDashboard.putData("line_PID", linePID);
        SmartDashboard.putData("line_motor", lineMotor);
    }

    public void manualControlLoop(double manualSpeed) {
        lineMotor.set(manualSpeed);
        linePID.setSetpoint(getLineLength());

        updateDashboard();
    }

    public void pidControlLoop() {
        var lineVolt = linePID.calculate(getLineLength());
        if (Math.abs(lineVolt) > 10) {
            lineVolt = lineVoltLimit * (lineVolt > 0 ? 1 : -1);
        }
        lineMotor.setVoltage(lineVolt);

        updateDashboard();
    }

    protected void updateDashboard() {
        SmartDashboard.putNumber("line_volt", lineMotor.getMotorOutputVoltage());
    }

    // setpoint
    public void setSetpoint(double setpoint) {
        final var currentSetpoint = getSetpoint();
        if (isPhyLimitExceed(currentSetpoint) != 0) {
            // if current setpoint exceed physical limit, don't do anything.
            return;
        }

        if (isPhyLimitExceed(currentSetpoint) == -1) {
            setpoint = lineLenghtMin;
        } else if (isPhyLimitExceed(currentSetpoint) == 1) {
            setpoint = lineLenghtMax;
        }

        linePID.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return linePID.getSetpoint();
    }

    // encoder
    public void resetEncoder() {
        lineMotor.setSelectedSensorPosition(0.0);
    }

    public double getLineLength() {
        double x = lineMotor.getSelectedSensorPosition();
        double length = 0.00473 * x - 0.0000000348 * x * x;
        return length;
    }

    // phy limit check
    protected int isPhyLimitExceed(double len) {
        return len < lineLenghtMin ? -1 : (len > lineLenghtMax ? 1 : 0);
    }
}
