package frc.robot.System;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.component.DistanceSensor;
import frc.robot.component.DriveBase;
import frc.robot.component.Intake;
import frc.robot.component.Joint;

public class NewAutoEngine {

    static int currentStep = 0;
    static int trajectoryAmount = 6;
    public static final int[] blueLeft = { 0 };
    public static final int[] blueMiddle = { 1 };
    public static final int[] blueRight = { 2 };
    public static final int[] redLeft = { 3 };
    public static final int[] redMiddle = { 4 };
    public static final int[] redRight = { 5 };
    public static final String[] trajJSON = {
            "/home/lvuser/deploy/output/output/BL.wpilib.json", "/home/lvuser/deploy/output/output/BM.wpilib.json",
            "/home/lvuser/deploy/output/output/BR.wpilib.json", "/home/lvuser/deploy/output/output/RL.wpilib.json",
            "/home/lvuser/deploy/output/output/RM.wpilib.json", "/home/lvuser/deploy/output/output/RR.wpilib.json"
    };

    static Trajectory[] trajectory = new Trajectory[trajectoryAmount];
    private static final String DoNothing = "DoNothing";
    private static final String BlueLeft = "BlueLeft";
    private static final String BlueMiddle = "BlueMiddle";
    private static final String BlueRight = "BlueRight";
    private static final String RedLeft = "RedLeft";
    private static final String RedMiddle = "RedMiddle";
    private static final String RedRight = "RedRight";

    // The string of the timer
    private static final String SideTimer = "SideTimer";
    private static final String MiddleTimer = "MiddleTimer";
    private static final String GoBackTimer = "GoBackTimer";

    public static Timer timer = new Timer();

    public static SendableChooser<String> chooser;
    public static String autoSelected;

    private static PIDController gyroPID;
    private static double kP = 0.42;
    private static double kI = 0;
    private static double kD = 0.03;
    private static boolean mode = false;

    public static void init() {

        chooser = new SendableChooser<String>();
        gyroPID = new PIDController(kP, kI, kD);

        putChooser();

        for (int i = 0; i < trajectoryAmount; i++) {
            try {
                // Importing PathWeaver JSON
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON[i]);
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory" + trajJSON[i] + "\n" + ex.getMessage(),
                        ex.getStackTrace());
            }

            var trajInitialPose = trajectory[i].getInitialPose();

            DriveBase.setODOPose(trajInitialPose);
        }
    }

    public static void start() {
        currentStep = 0;

        autoSelected = chooser.getSelected();

        DriveBase.resetEncoder();
        DriveBase.resetGyro();
        DriveBase.resetPID();

        double autoDelayTime = SmartDashboard.getNumber("auto_Delay", 0);

        Timer.delay(autoDelayTime);
        timer.reset();
        timer.start();
    }

    public static void loop() {

        DriveBase.updateODO();
        DriveBase.putDashboard();

        SmartDashboard.putNumber("auto_Timer", timer.get());
        SmartDashboard.putNumber("current_Step", currentStep);

        // SmartDashboard.putNumber("path_length",
        //         DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()));

        switch (autoSelected) {
            case DoNothing:
                DriveBase.directControl(0, 0);
                break;
            case BlueLeft:
                DoBlueLeft();
                break;
            case BlueMiddle:
                DoBlueMiddle();
                break;
            case BlueRight:
                DoBlueRight();
                break;
            case RedLeft:
                DoRedLeft();
                break;
            case RedMiddle:
                DoRedMiddle();
                break;
            case RedRight:
                DoRedRight();
                break;
            case SideTimer:
                DoLeftRightTimer();
                break;
            case MiddleTimer:
                DoMiddleTimer();
                break;
            case GoBackTimer:
                DoGoBackTimer();
                break;
            default:
        }
    }

    protected static void putChooser() {
        chooser.setDefaultOption("doNothing", DoNothing);
        chooser.addOption("blueLeft", BlueLeft);
        chooser.addOption("blueMiddle", BlueMiddle);
        chooser.addOption("blueRight", BlueRight);
        chooser.addOption("redLeft", RedLeft);
        chooser.addOption("redMiddle", RedMiddle);
        chooser.addOption("redRight", RedRight);

        chooser.addOption("sideTimer", SideTimer);
        chooser.addOption("middleTimer", MiddleTimer);
        chooser.addOption("goBack", GoBackTimer);
        SmartDashboard.putData(chooser);
    }

    public static void DoBlueLeft() {
        switch (currentStep) {
            case 0:
                autoArmControl(2, 1);
                if (Robot.arm.getAngleDegree() > 35.2 && Robot.arm.getAngleDegree() < 36.2) {
                    if (Robot.arm.getLength() > 129. && Robot.arm.getLength() < 133) {
                        currentStep++;
                    }
                }
                DriveBase.directControl(0, 0);
                break;
            case 1:
                Intake.solOn();
                currentStep++;
                break;
            case 2:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                // path
                DriveBase.odometry.resetPosition(trajectory[blueLeft[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueLeft[0]].getInitialPose());
                currentStep++;
                break;
            case 3:
                // Run the path of blueLeft
                DriveBase.runTraj(trajectory[blueLeft[0]], timer.get());
                if (timer.get() > trajectory[blueLeft[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
                DriveBase.directControl(0, 0);
        }
    }

    public static void DoBlueMiddle() {
        switch (currentStep) {
            case 0:
                autoArmControl(2, 1);
                if (Robot.arm.getAngleDegree() > 35.2 && Robot.arm.getAngleDegree() < 36.2) {
                    if (Robot.arm.getLength() > 129 && Robot.arm.getLength() < 133) {
                        currentStep++;
                    }
                }
                DriveBase.directControl(0, 0);
                break;
            case 1:
                Intake.solOn();
                currentStep++;
                break;
            case 2:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[blueMiddle[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueMiddle[0]].getInitialPose());
                currentStep++;
                break;
            case 3:
                // Run the second path of blueMiddle
                DriveBase.runTraj(trajectory[blueMiddle[0]], timer.get());
                if (timer.get() > trajectory[blueMiddle[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.directControl(0, 0);

        }
    }

    public static void DoBlueRight() {
        switch (currentStep) {
            case 0:
                autoArmControl(2, 1);
                if (Robot.arm.getAngleDegree() > 35.2 && Robot.arm.getAngleDegree() < 36.2) {
                    if (Robot.arm.getLength() > 129 && Robot.arm.getLength() < 133) {
                        currentStep++;
                    }
                }
                DriveBase.directControl(0, 0);
                break;
            case 1:
                Intake.solOn();
                currentStep++;
                break;
            case 2:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[blueRight[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueRight[0]].getInitialPose());
                currentStep++;
                break;
            case 3:
                // Run the second path of blueRight
                DriveBase.runTraj(trajectory[blueRight[0]], timer.get());
                if (timer.get() > trajectory[blueRight[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
                DriveBase.directControl(0, 0);
        }
    }

    public static void DoRedLeft() {
        switch (currentStep) {
            case 0:
                autoArmControl(2, 1);
                if (Robot.arm.getAngleDegree() > 35.2 && Robot.arm.getAngleDegree() < 36.2) {
                    if (Robot.arm.getLength() > 129 && Robot.arm.getLength() < 133) {
                        currentStep++;
                    }
                }
                DriveBase.directControl(0, 0);
                break;
            case 1:
                Intake.solOn();
                currentStep++;
                break;
            case 2:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[redLeft[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redLeft[0]].getInitialPose());
                currentStep++;
                break;
            case 3:
                // Run the second path of redLeft
                DriveBase.runTraj(trajectory[redLeft[0]], timer.get());
                if (timer.get() > trajectory[redLeft[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
                DriveBase.directControl(0, 0);
        }
    }

    public static void DoRedMiddle() {
        switch (currentStep) {
            case 0:
                autoArmControl(2, 1);
                if (Robot.arm.getAngleDegree() > 35.2 && Robot.arm.getAngleDegree() < 36.2) {
                    if (Robot.arm.getLength() > 129 && Robot.arm.getLength() < 133) {
                        currentStep++;
                    }
                }
                DriveBase.directControl(0, 0);
                break;
            case 1:
                Intake.solOn();
                currentStep++;
                break;
            case 2:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[redMiddle[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redMiddle[0]].getInitialPose());
                currentStep++;
                break;
            case 3:
                // Run the second path of redMiddle
                DriveBase.runTraj(trajectory[redMiddle[0]], timer.get());
                if (timer.get() > trajectory[redMiddle[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.directControl(0, 0);
        }
    }

    public static void DoRedRight() {
        switch (currentStep) {
            case 0:
                autoArmControl(2, 1);
                if (Robot.arm.getAngleDegree() > 35.2 && Robot.arm.getAngleDegree() < 36.2) {
                    if (Robot.arm.getLength() > 82 && Robot.arm.getLength() < 87) {
                        currentStep++;
                    }
                }
                DriveBase.directControl(0, 0);
                break;
            case 1:
                Intake.solOn();
                currentStep++;
                break;
            case 2:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[redRight[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redRight[0]].getInitialPose());
                currentStep++;
                break;
            case 3:
                // Run the second path of redRight
                DriveBase.runTraj(trajectory[redRight[0]], timer.get());
                if (timer.get() > trajectory[redRight[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);// need to test
                DriveBase.directControl(0, 0);
        }
    }

    // Autonomous written with timer
    public static void DoLeftRightTimer() {
        double leftWheelVoltage = 0.8;
        double rightWheelVoltage = 0.8;
        if (timer.get() <= 4) {
            autoArmControl(2, 1);
            DriveBase.directControl(0, 0);
        } else if (timer.get() > 4 && timer.get() <= 7) {
            // Intake.solOn();
            autoArmControl(2, 0);
        } else if (timer.get() > 7 && timer.get() <= 8.5) {
            autoArmControl(5, 0);
            //DriveBase.directControl(-leftWheelVoltage, -rightWheelVoltage);
        } else if (timer.get() > 8.5 && timer.get() <= 9) {
            if (DistanceSensor.getDistence() < 19) {
                DriveBase.directControl(0, 0);
                Intake.solOff();
            } else {
                DriveBase.directControl(-0.3, -0.3);
            }
        } else if (timer.get() > 9 && timer.get() <= 13.1) {
            autoArmControl(2, 0);
            DriveBase.directControl(-leftWheelVoltage, -rightWheelVoltage);
        } else if (timer.get() > 13.1 && timer.get() < 14.5) {
            DriveBase.directControl(0, 0);
            autoArmControl(2, 1);
        } else {
            Intake.solOn();
        }
        
    }

    public static void DoMiddleTimer() {
        if (timer.get() <= 5) {
            autoArmControl(2, 1);
            DriveBase.directControl(0, 0);
        } else if (timer.get() > 5 && timer.get() <= 5.3) {
            Intake.solOn();
         } else if (timer.get() <= 7) {
            doMiddle();
        }
    }

    public static void DoGoBackTimer() {
        double leftWheelVoltage = 0.8;
        double rightWheelVoltage = 0.8;
        if (timer.get() <= 4) {
            autoArmControl(2, 1);
            DriveBase.directControl(0, 0);
        }else if (timer.get() >4 && timer.get()<= 8) {
            DriveBase.directControl(leftWheelVoltage, rightWheelVoltage);
        } else {
            DriveBase.directControl(0, 0);
        }
        DriveBase.putDashboard();
    }

    public static int autoArmControl(int modeArm, int modeLine) {
        switch (modeArm) {
            case 0: // the beginning position
                Robot.arm.setAngleSetPoint(68.5);
                break;
            case 1: // the first level
                Robot.arm.setAngleSetPoint(-10);
                break;
            case 2: // the second level and the third level
                Robot.arm.setAngleSetPoint(35.7);
                break;
            case 4: // the beginning position of the other side of the robot
                Robot.arm.setAngleSetPoint(111.5);
                break;
            case 5: // the first level of the other side of the robot
                Robot.arm.setAngleSetPoint(160);
                break;
            case 6:// the second level of the other side of the robot
                Robot.arm.setAngleSetPoint(140.8);
                break;
            default:
                break;
        }

        switch (modeLine) {
            case 0: // the beginning position
                Robot.arm.setLineSetPoint(40);
                break;
            case 1: // the second level
                Robot.arm.setLineSetPoint(89.8);
                break;
            case 2: // the third level
                Robot.arm.setLineSetPoint(130);
                break;
            default:
                break;
        }

        Robot.arm.autoArmLoop();
        return 0;
    }

    public static void doMiddle() {
        double degree = DriveBase.getGyroDegree();
        if (degree >= 10) {
            goChargeStation();
            if(degree>=-0.1 && degree<=0.1){
                DriveBase.directControl(0, 0);
            }
        } else if (!mode) {
            DriveBase.directControl(0.6, 0.6);
        }
        SmartDashboard.getNumber("pitch", degree);
    }

    public static void goChargeStation() {
        mode = true;
        gyroPID.setSetpoint(0);
        double driveDegree = DriveBase.getGyroDegree();
        double driveSpeed = -gyroPID.calculate(driveDegree);
        if (driveDegree >= 0.6) {
            driveSpeed = 0.6;
        } else if (driveDegree <= -0.6) {
            driveSpeed = -0.6;
        }
        DriveBase.directControl(driveSpeed, driveSpeed);
    }
}
