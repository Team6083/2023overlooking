package frc.robot.component;

import javax.swing.text.AbstractDocument.LeafElement;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
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
    private final double armAngleMin = -15;
    private final double armAngleMax = 185;

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

        SmartDashboard.putData(armPID);
        SmartDashboard.putData(armMotor);
    }

    // control loops
    public void armInManualControlLoop(double manualSpeed) {
        armMotor.set(manualSpeed);
        armPID.setSetpoint(getAngleDegree());
    }

    public void pidControlLoop() {
        var armVolt = armPID.calculate(getAngleDegree());

    }

    // encoder get angle
    public double getAngleDegree() {
        return getSparkMaxAngleDegree();
    }

    private double getSparkMaxAngleDegree() {
        return (sparkMaxEncoder.getPosition() * 360 / armEncoderGearing) + angleDegreeOffset;
    }
    private double getRevEncoderAngleDegree(){
        return (revEncoder.get()*360/armEncoderPulse)+angleDegreeOffset;
    }

    // reset encoder
    public void resetEncoder() {
        angleDegreeOffset = 0;
        resetSparkMaxEncoder();
        resetRevEncoder();
    }

    public void resetSparkMaxEncoder() {
        sparkMaxEncoder.setPosition(0);
    }

    public void resetRevEncoder() {
        revEncoder.reset();
    }
}
