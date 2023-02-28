package frc.robot.hardware.armEncoder;

import edu.wpi.first.wpilibj.Encoder;

public class RevHexArmEncoder implements ArmEncoder {

    private Encoder encoder;

    private static final double armEncoderPulse = 2048;

    public RevHexArmEncoder(int channelA, int channelB) {
        encoder = new Encoder(channelA, channelB);
        encoder.setReverseDirection(true);
        encoder.setDistancePerPulse(360 / armEncoderPulse);
    }

    @Override
    public double getAngleDegree() {
        return encoder.getDistance();
    }

    @Override
    public double getRawPosition() {
        return encoder.getRaw();
    }

    @Override
    public void reset() {
        encoder.reset();
    }

}
