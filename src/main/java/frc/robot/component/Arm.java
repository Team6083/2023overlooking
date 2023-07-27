package frc.robot.component;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Arm {
    protected Joint joint;
    protected Line line;

    public static final double firstStageToJoint = 50;

    public static final double extendLimit = 120; // rule: 122
    public static final double jointToFrameDist = 40;

    public static final double heightLimit = 150; // rule: 198
    public static final double jointHeight = 40; // dist from joint to floor

    public Arm() {
        joint = new Joint(68.5);
        line = new Line(40);
    }

    public void teleop(XboxController mainController, XboxController viceController) {
        // arm encoder reset
        putDashboard();
        if (mainController.getBackButton()) {
            joint.resetEncoder();
            joint.resetSetpoint();
        }

        boolean inPolar = mainController.getBButton();
        int backButtonPressed = viceController.getBackButton() ? 1 : 0;

        double newJointSetpoint = joint.getSetpoint();
        double newLineSetpoint = line.getPIDSetpoint();

        if (inPolar) { // Control using polar coordinate.
            newJointSetpoint += (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis()) * -0.7;
            if (mainController.getPOV() == 0) {
                newLineSetpoint += 0.4;
            } else if (mainController.getPOV() == 180) {
                newLineSetpoint += -0.5;
            }
        } else { // Control using Cartesian coordinate.
            var currCord = polarToCartes(newJointSetpoint, newLineSetpoint);

            if (mainController.getPOV() != -1) {
                double newX = currCord[0] + Math.cos(Math.toRadians(mainController.getPOV())) * 10;
                double newY = currCord[1] + Math.sin(Math.toRadians(mainController.getPOV())) * 10;

                var tmp = cartesToPolar(newX, newY);
                newLineSetpoint = tmp[0];
                newJointSetpoint = tmp[1];
            }
        }

        if (viceController.getRightBumper() || viceController.getLeftBumper()) {
            newJointSetpoint = Joint.armAngleSetpoints[backButtonPressed][0];
        } else if (viceController.getPOV() == 270) {
            newJointSetpoint = Joint.armAngleSetpoints[backButtonPressed][2];
        } else if (viceController.getBButton()) {
            newJointSetpoint = Joint.armAngleSetpoints[backButtonPressed][1];
        } else if (viceController.getPOV() == 90) {
            newJointSetpoint = Joint.armAngleSetpoints[backButtonPressed][3];
        }

        // line encoder reset
        if (mainController.getStartButton()) {
            line.resetEncoder();
            line.resetSetpoint();
        }

        if (viceController.getLeftBumper()) {
            newLineSetpoint = 84.6;
        } else if (viceController.getRightBumper()) {
            newLineSetpoint = 131;
        } else if (viceController.getBButton()) {
            newLineSetpoint = 98.14;
        } else if (viceController.getPOV() == 270 || viceController.getPOV() == 90) {
            newLineSetpoint = 40;
        }

        joint.setSetpoint(newJointSetpoint);
        line.setPIDSetpoint(newLineSetpoint);

        // limit line length by joint angle
        double jointAngleRadian = Math.toRadians(joint.getAngleDegree());
        final double lineLengthLimit = getLineMaxLengthByJointAngle(jointAngleRadian);
        if (lineLengthLimit > 0 && line.getPIDSetpoint() > lineLengthLimit) {
            line.setPIDSetpoint(lineLengthLimit);
        }

        SmartDashboard.putNumber("line_length_limit", lineLengthLimit);

        // Joint and Line control loop
        boolean jointInManual = (mainController.getAButton());
        if (jointInManual) {
            double rotatePower = (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis()) * -0.15;
            joint.armInManualControlLoop(rotatePower);
        } else {
            joint.pidControlLoop();
        }

        boolean lineInManual = (mainController.getXButton());
        final double lineMotorCurrentLimit = 10;
        double lineMotorCurrent = Robot.pd.getCurrent(0);
        if (lineMotorCurrent > lineMotorCurrentLimit) {
            line.stopMotor();
        } else {
            if (lineInManual) {
                if (mainController.getPOV() == 0) {
                    line.manualControlLoop(0.3);
                } else if (mainController.getPOV() == 180) {
                    line.manualControlLoop(-0.3);
                } else {
                    line.manualControlLoop(0);
                }
            } else {
                line.pidControlLoop();
            }
        }
    }

    public double getAngleDegree() {
        return joint.getAngleDegree();
    }

    public double getLength() {
        return line.getLineLength();
    }

    public void setAngleSetPoint(double angleSetPoint) {
        joint.setSetpoint(angleSetPoint);
        return;
    }

    public void setLineSetPoint(double lineSetPoint) {
        line.setPIDSetpoint(lineSetPoint);
        return;
    }

    public void autoArmLoop() {
        putDashboard();
        joint.pidControlLoop();
        line.pidControlLoop();
    }

    public void putDashboard() {
        SmartDashboard.putNumber("arm_angle", getAngleDegree());
        SmartDashboard.putNumber("line_length", getLength());
    }

    public double getLineMaxLengthByJointAngle(double jointAngleRadian) {
        double angle = jointAngleRadian;

        double maxProj = jointToFrameDist + extendLimit;
        double maxLenByExtendLimit = (maxProj / Math.abs(Math.cos(angle))) - firstStageToJoint;

        double maxLenByHeightLimit = ((heightLimit - jointHeight) / Math.abs(Math.sin(angle))) - firstStageToJoint;

        return Math.min(maxLenByExtendLimit, maxLenByHeightLimit);
    }

    /**
     * Transform Cartesian coord. to polar coord.
     */
    public double[] cartesToPolar(double x, double y) {
        double lineLength = Math.sqrt(x * x + y * y);
        double angle = Math.acos(x / lineLength);
        double tmp[] = { lineLength, Math.toDegrees(angle) };
        return tmp;
    }

    /**
     * Transform polar coord. to Cartesian coord.
     */
    public double[] polarToCartes(double lineLength, double angleDegree) {
        double x = Math.cos(Math.toRadians(angleDegree)) * lineLength;
        double y = Math.sin(Math.toRadians(angleDegree)) * lineLength;
        double tmp[] = { x, y };
        return tmp;
    }
}
