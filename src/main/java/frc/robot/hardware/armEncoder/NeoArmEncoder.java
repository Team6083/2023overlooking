package frc.robot.hardware.armEncoder;

import com.revrobotics.RelativeEncoder;

public class NeoArmEncoder implements ArmEncoder {

    private static final double armEncoderGearing = 198;

    private RelativeEncoder encoder;

    public NeoArmEncoder(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    @Override
    public double getAngleDegree() {
        return getRawPosition() * 360 / armEncoderGearing;
    }

    @Override
    public double getRawPosition() {
        return encoder.getPosition();
    }

    @Override
    public void reset() {
        setPosition(0);
    }

    public void setPosition(double pos) {
        encoder.setPosition(pos);
    }

}
