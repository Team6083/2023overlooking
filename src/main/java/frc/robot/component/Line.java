package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;

public class Line {
    // line motor
    private static final int lineId = 17;
    private static WPI_TalonSRX lineMotor;

    // PID controller for line motor
    private static double kLP = 0.35;
    private static double kLI = 0.0;
    private static double kLD = 0.0;
    protected static PIDController linePID;

    // constants for line motor
    private static final double modifiedLineVoltLimit = 3;
    private static final double maxLineLengthLimit = 140;
    private static final double minLineLengthLimit = 40;

    public Line(double lineInitSensorPosition, double linePIDInitSetPoint) {
        lineMotor = new WPI_TalonSRX(lineId);
        lineMotor.setInverted(true);
        lineMotor.setSensorPhase(true);
        lineMotor.setSelectedSensorPosition(lineInitSensorPosition);

        linePID = new PIDController(kLP, kLI, kLD);
        linePID.setSetpoint(linePIDInitSetPoint);
    }

    public void manualControlLoop(double manualControlSpeed) {
        lineMotor.set(manualControlSpeed);
        linePID.setSetpoint(getLineLength());
    }

    public void PIDControlLoop() {
        double lineVolt = linePID.calculate(getLineLength());
        if (Math.abs(lineVolt) > modifiedLineVoltLimit) {
            lineVolt = modifiedLineVoltLimit * (lineVolt > 0 ? 1 : -1);
        }
        lineMotor.setVoltage(lineVolt);
    }

    public double getPIDSetpoint() {
        return linePID.getSetpoint();
    }

    public void setPIDSetpoint(double setpoint) {
        if (setpoint < minLineLengthLimit) {
            setpoint = minLineLengthLimit;
        } else if (setpoint > maxLineLengthLimit) {
            setpoint = maxLineLengthLimit;
        }
        linePID.setSetpoint(setpoint);
    }

    public void resetEncoderPosAndSetpoint() {
        lineMotor.setSelectedSensorPosition(0);
        linePID.setSetpoint(40);
    }

    public double getLineLength() {
        double x = lineMotor.getSelectedSensorPosition();
        double cal1 = 0.00473 * x;
        double cal2 = 0.0000000348 * x * x;
        double length = cal1 - cal2 + 40;
        return length;
    }
}
