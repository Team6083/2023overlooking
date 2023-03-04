package frc.robot.component;

import edu.wpi.first.wpilibj.XboxController;

public class ArmAndLine {

    protected Line line;
    protected Arm arm;

    public ArmAndLine() {
        arm = new Arm(68.5);
        line = new Line(0);
    }

    public void teleop(XboxController mainC, XboxController viceC) {

        // arm encoder reset
        if (mainC.getBackButton()) {
            arm.resetEncoder();
        }

        if (viceC.getRightBumper() || viceC.getLeftBumper()) {
            arm.setSetpoint(35.2);
        } else if (viceC.getPOV() == 270) {
            arm.setSetpoint(68.5);
        } else if (viceC.getBButton()) {
            arm.setSetpoint(28.38);
        } else if (viceC.getYButton()) {
            arm.setSetpoint(90);
        } else {
            double armAngleModify = (mainC.getLeftTriggerAxis() - mainC.getRightTriggerAxis()) * -0.3;
            arm.setSetpoint(arm.getSetpoint() + armAngleModify);
        }

        final boolean armInManual = mainC.getAButton();
        if (armInManual) {
            // control through xbox, for test
            double rotateSpeed = (mainC.getLeftTriggerAxis() - mainC.getRightTriggerAxis()) * -0.15;
            arm.manualControlLoop(rotateSpeed);
        } else {
            arm.pidControlLoop();
        }

        /*
         * Line begin
         */

        // line encoder reset
        if (mainC.getStartButton()) {
            line.resetEncoder();
        }

        // line to specific len
        if (viceC.getLeftBumper()) {
            line.setSetpoint(33.02);
        } else if (viceC.getRightBumper()) {
            line.setSetpoint(86.15);
        } else if (viceC.getBButton()) {
            line.setSetpoint(58.14);
        }

        // adjust line len
        if (mainC.getPOV() == 0) {
            line.setSetpoint(line.getSetpoint() + 0.3);
        } else if (mainC.getPOV() == 180) {
            line.setSetpoint(line.getSetpoint() - 0.4);
        }

        final boolean lineInManual = mainC.getXButton();
        if (lineInManual) {
            double linePower = 0;
            if (mainC.getPOV() == 0) {
                linePower = 0.3;
            } else if (mainC.getPOV() == 180) {
                linePower = -0.3;
            }

            line.manualControlLoop(linePower);
        } else {
            line.pidControlLoop();
        }
    }

}
