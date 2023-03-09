package frc.robot.component;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm {
    private final int armLeftCANId = 15;
    private final int armRightCANId = 16;

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
     
    public Arm() {
        armMotorLeft = new CANSparkMax(armLeftCANId, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(armRightCANId, MotorType.kBrushless);
        armMotor = new MotorControllerGroup(armMotorLeft, armMotorRight);
        armMotorLeft.setInverted(true);

    }

}
