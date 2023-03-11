package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Joint {
    private final int armLeftCANId = 15;
    private final int armRightCANId = 16;
    private final int revEncoderChannel1 = 8;
    private final int revEncoderChannel2 = 9;

    private CANSparkMax armMotorLeft;
    private CANSparkMax armMotorRight;
    private MotorControllerGroup armMotor;

    // encoders
    private RelativeEncoder sparkMaxEncoder;
    private Encoder revEncoder;
    private double angleDegreeOffset;

    private PIDController armPID;
    private double kP = 0.3;

    private final double armEncoderPulse = 2048;
    private final double armEncoderGearing = 198;
    private final double armVoltLimit = 4;
    private final double armAngleMin = -20;
    private final double armAngleMax = 195;
    public static final double[][] armAngleSetpoints = { { 35.6, 28.38, 90, -10 }, { 130, 151, 90, 180 } };

    public Joint(double armInitAngleDegree) {
        armMotorLeft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorLeft, armMotorRight);
        armMotorLeft.setInverted(true);

        // init encoder
        sparkMaxEncoder = armMotorLeft.getEncoder();
        revEncoder = new Encoder(revEncoderChannel1, revEncoderChannel2);
        resetEncoder();
        angleDegreeOffset = armInitAngleDegree;

        armPID = new PIDController(kP, 0, 0);
        armPID.setSetpoint(armInitAngleDegree);

        SmartDashboard.putData("arm_PID", armPID);
        SmartDashboard.putData("arm_motor", armMotor);
    }

    // control loops
    public void armInManualControlLoop(double manualSpeed) {
        armMotor.set(manualSpeed);
        armPID.setSetpoint(getAngleDegree());
    }

    public void pidControlLoop() {
        var armVolt = armPID.calculate(getAngleDegree());

        double modifiedArmVolt = armVolt;
        if (Math.abs(modifiedArmVolt) > armVoltLimit) {
            modifiedArmVolt = armVoltLimit * (armVolt > 0 ? 1 : -1);
        }
        armMotor.setVoltage(modifiedArmVolt);

        SmartDashboard.putNumber("arm_volt", modifiedArmVolt);
    }

    // PID get setpoint
    public double getSetpoint() {
        return armPID.getSetpoint();
    }

    // PID set setpoint
    public void setSetpoint(double setpoint) {
        final var currentSetpoint = getSetpoint();
        if (isPhyLimitExceed(currentSetpoint) != 0) {
            // if current setpoint exceed physical limit, don't do anything.
            return;
        }

        if (isPhyLimitExceed(setpoint) == -1) {
            setpoint = armAngleMin;
        } else if (isPhyLimitExceed(setpoint) == 1) {
            setpoint = armAngleMax;
        }

        armPID.setSetpoint(setpoint);
    }

    // encoder get angle
    public double getAngleDegree() {
        return getSparkMaxAngleDegree(); // change between getSparkMaxAngleDegree and getRevEncoderAngleDegree. Depend
                                         // on which encoder you use.
    }

    private double getSparkMaxAngleDegree() {
        SmartDashboard.putNumber("jointEncoderPos", sparkMaxEncoder.getPosition());
        return (sparkMaxEncoder.getPosition() * 360 / armEncoderGearing) + angleDegreeOffset;
    }

    private double getRevEncoderAngleDegree() {
        return (revEncoder.get() * 360 / armEncoderPulse) + angleDegreeOffset;
    }

    // reset encoder
    public void resetEncoder() {
        angleDegreeOffset = 0;
        resetSparkMaxEncoder();
        resetRevEncoder();
    }

    private void resetSparkMaxEncoder() {
        sparkMaxEncoder.setPosition(0);
    }

    private void resetRevEncoder() {
        revEncoder.reset();
    }

    public void resetSetpoint() {
        armPID.setSetpoint(0);
    }

    private int isPhyLimitExceed(double angle) {
        return angle < armAngleMin ? -1 : (angle > armAngleMax ? 1 : 0);
    }
}
