package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {
    protected final int armLeftCANId = 15;
    protected final int armRightCANId = 16;

    protected CANSparkMax armMotorLeft;
    protected CANSparkMax armMotorRight;
    protected MotorControllerGroup armMotor;

    // encoders
    protected RelativeEncoder neoEnc;
    protected Encoder revEnc;

    // PID
    protected double kP = 0.3;
    protected PIDController armPID;

    protected static final double revArmEncoderPulse = 2048;
    protected static final double armEncoderGearing = 198;
    protected static final double armVoltLimit = 4;
    protected static final double armAngleMin = -15;
    protected static final double armAngleMax = 185;

    public Arm(double initSetpoint) {
        armMotorLeft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorLeft, armMotorRight);
        armMotorLeft.setInverted(true);

        // init NEO encoder
        neoEnc = armMotorLeft.getEncoder();

        // init REV encoder
        revEnc = new Encoder(8, 9);

        armPID = new PIDController(kP, 0, 0);

        SmartDashboard.putData("arm_PID", armPID);
    }

    public void manualControlLoop(double manualSpeed) {
        armMotor.set(manualSpeed);
        armPID.setSetpoint(getArmDegree());

        updateDashboard();
    }

    public void pidControlLoop() {
        var lineVolt = armPID.calculate(getArmDegree());
        if (Math.abs(lineVolt) > 10) {
            lineVolt = armVoltLimit * (lineVolt > 0 ? 1 : -1);
        }
        armMotor.setVoltage(lineVolt);


        SmartDashboard.putNumber("arm_volt", lineVolt);

        updateDashboard();
    }

    protected void updateDashboard() {
    }

    // setpoint
    public void setSetpoint(double setpoint) {
        final var currentSetpoint = getSetpoint();
        if (isPhyLimitExceed(currentSetpoint) != 0) {
            // if current setpoint exceed physical limit, don't do anything.
            return;
        }

        if (isPhyLimitExceed(currentSetpoint) == -1) {
            setpoint = armAngleMin;
        } else if (isPhyLimitExceed(currentSetpoint) == 1) {
            setpoint = armAngleMax;
        }

        armPID.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return armPID.getSetpoint();
    }

    // set encoder
    public void resetEncoder() {
        resetNEOEncoder();
    }

    protected void resetREVEncoder() {
        revEnc.reset();
    }

    protected void resetNEOEncoder() {
        neoEnc.setPosition(0);
    }

    // get encoder
    public double getArmDegree() {
        return getNEOArmDegree();
    }

    protected double getREVArmDegree() {
        return revEnc.get() * 360 / revArmEncoderPulse;
    }

    protected double getNEOArmDegree() {
        return neoEnc.getPosition() * 360 / armEncoderGearing;
    }

    // phy limit check
    protected int isPhyLimitExceed(double angle) {
        return angle < armAngleMin ? -1 : (angle > armAngleMax ? 1 : 0);
    }
}
