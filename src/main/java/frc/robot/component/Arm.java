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

    public static final double heightLimit = 155; // rule: 198
    public static final double jointHeight = 40; // dist from joint to floor

    //cartes controller use in cartes controller branch, not use in main branch
    /*
    public static double x = Math.cos(Math.toRadians(68.5)) * 40;
    public static double y = Math.sin(Math.toRadians(68.5)) * 40+jointHeight;
    */

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

        //cartes controller use in cartes controller branch, not use in main branch
        // boolean inPolar = mainController.getBButton();

        int backButtonPressed = viceController.getBackButton() ? 1 : 0;
        if (viceController.getRightBumper() || viceController.getLeftBumper()) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][0]);
        } else if (mainController.getPOV() == 90) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][2]);
        } else if (viceController.getBButton()) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][1]);
        } else if (viceController.getPOV() == 90) {
            joint.setSetpoint(Joint.armAngleSetpoints[backButtonPressed][3]);
        } else{
            double armAngleModify = (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis())
                    * -0.7;
            joint.setSetpoint(joint.getSetpoint() + armAngleModify);
        }

        // line encoder reset
        if (mainController.getStartButton()) {
            line.resetEncoder();
            line.resetSetpoint();
        }

        double lineLengthModify = 0.0;

        //cartes controller use in cartes controller branch, not use in main branch
        /* 
        double xModify = 0.0;
        double yModify = 0.0;
        */

        if (viceController.getLeftBumper()) {
            line.setPIDSetpoint(84.6);
        } else if (viceController.getRightBumper()) {
            line.setPIDSetpoint(131);
        } else if (viceController.getBButton()) {
            line.setPIDSetpoint(98.14);
        } else if (mainController.getPOV() == 90 || viceController.getPOV() == 90) {
            line.setPIDSetpoint(40);
        } else if (mainController.getPOV() == 0) {
            lineLengthModify = 0.4;
            // yModify = 0.5;
        } else if (mainController.getPOV() == 180) {
            lineLengthModify = -0.5;
            // yModify = -0.5;
        }

        //cartes controller use in cartes controller branch, not use in main branch
        /* 
        if (mainController.getPOV() == 90) {
            xModify = 0.5;
        } else if (mainController.getPOV() == 270) {
            xModify = -0.5;
        }
        
        if (inPolar) {
            line.setPIDSetpoint(line.getPIDSetpoint() + lineLengthModify);
        } else {
            double tmp[] = cartesToPolar(x + xModify, y + yModify);
            line.setPIDSetpoint(tmp[0]);
            joint.setSetpoint(tmp[1]);
        }
        */

        line.setPIDSetpoint(line.getPIDSetpoint() + lineLengthModify);

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
        double tmp[] = { lineLength, angle };
        return tmp;
    }
}
