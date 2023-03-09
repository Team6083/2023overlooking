package frc.robot.component;

import edu.wpi.first.wpilibj.XboxController;

public class Arm {
    protected Joint arm;
    protected Line line;

    public Arm() {
        arm = new Joint(68.5);
        line = new Line(40);
    }

    public void teleop(XboxController mainController, XboxController viceController) {
        // arm encoder reset
        if (mainController.getBackButton()) {
            arm.resetEncoder();
        }

        int backButtonPressed = viceController.getBackButton() ? 1 : 0;
        if (viceController.getLeftBumper() || viceController.getRightBumper()) {
            if (viceController.getRightBumper() || viceController.getLeftBumper()) {
                arm.setSetpoint(arm.armAngleSetpoints[backButtonPressed][0]);
            } else if (viceController.getPOV() == 270) {
                arm.setSetpoint(arm.armAngleSetpoints[backButtonPressed][0]);
            } else if (viceController.getBButton()) {
                arm.setSetpoint(arm.armAngleSetpoints[backButtonPressed][0]);
            } else if (viceController.getYButton()) {
                arm.setSetpoint(arm.armAngleSetpoints[backButtonPressed][0]);
            } else {
                double armAngleModify = (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis())
                        * -0.3;
                arm.setSetpoint(arm.getSetpoint() + armAngleModify);
            }
        }
        boolean armInManual = (mainController.getAButton());
        if (armInManual) {
            double rotatePower = (mainController.getLeftTriggerAxis() - mainController.getRightTriggerAxis()) * -0.15;
            arm.armInManualControlLoop(rotatePower);
        } else {
            arm.pidControlLoop();
        }


        // line encoder reset
        if (mainController.getStartButton()) {
            line.resetEncoder();
        }

        double lineLengthModify = 0.0;
        if (viceController.getLeftBumper()) {
            line.setPIDSetpoint(84.6);
        } else if (viceController.getRightBumper()) {
            line.setPIDSetpoint(131);
        } else if (viceController.getBButton()) {
            line.setPIDSetpoint(98.14);
        } else if (mainController.getPOV() == 0) {
            lineLengthModify = 0.3;
            line.setPIDSetpoint(line.getPIDSetpoint() + lineLengthModify);
        } else if (mainController.getPOV() == 180) {
            lineLengthModify = -0.4;
            line.setPIDSetpoint(line.getPIDSetpoint() + lineLengthModify);
        }
        boolean lineInManual = (mainController.getXButton());
        final double lineMotorCurrentLimit = 10;
        double lineMotorCurrent = line.getLineLength();
        if (lineMotorCurrent > lineMotorCurrentLimit) {
            line.lineMotor.stopMotor();
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
    double radian = Math.toRadians(arm.getAngleDegree());
        if (line.getPIDSetpoint() > 170 * Math.abs(1 / Math.cos(radian)) - 60) {
            line.setPIDSetpoint( 170 * Math.abs(1 / Math.cos(radian))-60); 
        }
        line.setPIDSetpoint( 170 * Math.abs(1 / Math.cos(radian))-60);
        
}}
