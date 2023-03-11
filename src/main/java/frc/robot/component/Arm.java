package frc.robot.component;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Arm {
    protected Joint joint;
    protected Line line;

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

        int backButtonPressed = viceController.getBackButton() ? 1 : 0;
        if (viceController.getRightBumper() || viceController.getLeftBumper()) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][0]);
        } else if (viceController.getPOV() == 270) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][2]);
        } else if (viceController.getBButton()) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][1]);
        } else if (viceController.getPOV() == 90) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][3]);
        } else {
            double armAngleModify = (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis())
                    * -0.7;
            joint.setSetpoint(joint.getSetpoint() + armAngleModify);
        }
        boolean armInManual = (mainController.getAButton());
        if (armInManual) {
            double rotatePower = (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis()) * -0.15;
            joint.armInManualControlLoop(rotatePower);
        } else {
            joint.pidControlLoop();
        }

        // line encoder reset
        if (mainController.getStartButton()) {
            line.resetEncoder();
            line.resetSetPoint();
        }

        double lineLengthModify = 0.0;
        if (viceController.getLeftBumper()) {
            line.setPIDSetpoint(84.6);
        } else if (viceController.getRightBumper()) {
            line.setPIDSetpoint(131);
        } else if (viceController.getBButton()) {
            line.setPIDSetpoint(98.14);
        } else if (viceController.getPOV() == 270 || viceController.getPOV() == 90) {
            line.setPIDSetpoint(40);
        } else if (mainController.getPOV() == 0) {
            lineLengthModify = 0.4;
            line.setPIDSetpoint(line.getPIDSetpoint() + lineLengthModify);
        } else if (mainController.getPOV() == 180) {
            lineLengthModify = -0.5;
            line.setPIDSetpoint(line.getPIDSetpoint() + lineLengthModify);
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
                line.PIDControlLoop();
            }
        }
        // double radian = Math.toRadians(joint.getAngleDegree());
        // if (line.getPIDSetpoint() > 170 * Math.abs(1 / Math.cos(radian)) - 60) {
        // line.setPIDSetpoint(170 * Math.abs(1 / Math.cos(radian)) - 60);
        // }
        // line.setPIDSetpoint(170 * Math.abs(1 / Math.cos(radian)) - 60);

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
        line.PIDControlLoop();
    }

    public void putDashboard() {
        SmartDashboard.putNumber("arm_angle", getAngleDegree());
        SmartDashboard.putNumber("line_length", getLength());
    }
}
