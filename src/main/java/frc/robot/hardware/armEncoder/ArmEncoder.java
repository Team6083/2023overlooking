package frc.robot.hardware.armEncoder;

public interface ArmEncoder {
    double getAngleDegree();

    double getRawPosition();

    void reset();
}
