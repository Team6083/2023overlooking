package frc.robot.System;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.component.Arm;
import frc.robot.component.DriveBase;
import frc.robot.component.Intake;

public class NewAutoEngine {

    static int currentStep = 0;
    static int trajectoryAmount = 12;
    public static final int[] blueLeft = { 0, 1 };
    public static final int[] blueMiddle = { 2, 3 };
    public static final int[] blueRight = { 4, 5 };
    public static final int[] redLeft = { 6, 7 };
    public static final int[] redMiddle = { 8, 9 };
    public static final int[] redRight = { 10, 11 };
    public static final String[] trajJSON = {
            "/home/lvuser/deploy/output/output/BL-1.wpilib.json", "/home/lvuser/deploy/output/output/BL-2.wpilib.json",
            "/home/lvuser/deploy/output/output/BM-1.wpilib.json", "/home/lvuser/deploy/output/output/BM-2.wpilib.json",
            "/home/lvuser/deploy/output/output/BR-1.wpilib.json", "/home/lvuser/deploy/output/output/BR-2.wpilib.json",
            "/home/lvuser/deploy/output/output/RL-1.wpilib.json", "/home/lvuser/deploy/output/output/RL-2.wpilib.json",
            "/home/lvuser/deploy/output/output/RM-1.wpilib.json", "/home/lvuser/deploy/output/output/RM-2.wpilib.json",
            "/home/lvuser/deploy/output/output/RR-1.wpilib.json", "/home/lvuser/deploy/output/output/RR-2.wpilib.json"
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
    private static final String GoBackWithTimer = "GoBackWithTimer";

    public static Timer timer = new Timer();

    public static SendableChooser<String> chooser;
    public static String autoSelected;

    private static double lastTime;
    private static double lastDegree;

    public static void init() {

        chooser = new SendableChooser<String>();

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
            case GoBackWithTimer:
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
        chooser.addOption("goBack", GoBackWithTimer);
        SmartDashboard.putData(chooser);
    }

    public static void DoBlueLeft() {
        switch (currentStep) {
            case 0:
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[blueLeft[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueLeft[0]].getInitialPose());
                currentStep++;
                break;
            case 1:
                // Run the first path of blueLeft
                DriveBase.runTraj(trajectory[blueLeft[0]], timer.get());
                if (timer.get() > trajectory[blueLeft[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            case 2:
                autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoder();
                DriveBase.leftMotor1.setInverted(false);
                DriveBase.rightMotor1.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[blueLeft[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueLeft[1]].getInitialPose());
                currentStep++;
                break;
            case 4:
                // Run the second path of blueLeft
                DriveBase.runTraj(trajectory[blueLeft[1]], timer.get());
                if (timer.get() > trajectory[blueLeft[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
        }
    }

    public static void DoBlueMiddle() {
        switch (currentStep) {
            case 0:
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[blueMiddle[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueMiddle[0]].getInitialPose());
                currentStep++;
                break;
            case 1:
                // Run the first path of blueMiddle
                DriveBase.runTraj(trajectory[blueMiddle[0]], timer.get());
                if (timer.get() > trajectory[blueMiddle[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            case 2:
                autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.resetEncoder();
                DriveBase.leftMotor1.setInverted(false);
                DriveBase.rightMotor1.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[blueMiddle[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueMiddle[1]].getInitialPose());
                currentStep++;
                break;
            case 4:
                // Run the second path of blueMiddle
                DriveBase.runTraj(trajectory[blueMiddle[1]], timer.get());
                if (timer.get() > trajectory[blueMiddle[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
        }
    }

    public static void DoBlueRight() {
        switch (currentStep) {
            case 0:
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[blueRight[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueRight[0]].getInitialPose());
                currentStep++;
                break;
            case 1:
                // Run the first path of blueRight
                DriveBase.runTraj(trajectory[blueRight[0]], timer.get());
                if (timer.get() > trajectory[blueRight[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            case 2:
                autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoder();
                DriveBase.leftMotor.setInverted(false);
                DriveBase.rightMotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[blueRight[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[blueRight[1]].getInitialPose());
                currentStep++;
                break;
            case 4:
                // Run the second path of blueRight
                DriveBase.runTraj(trajectory[blueRight[1]], timer.get());
                if (timer.get() > trajectory[blueRight[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
        }
    }

    public static void DoRedLeft() {
        switch (currentStep) {
            case 0:
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[redLeft[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redLeft[0]].getInitialPose());
                currentStep++;
                break;
            case 1:
                // Run the first path of redLeft
                DriveBase.runTraj(trajectory[redLeft[0]], timer.get());
                if (timer.get() > trajectory[redLeft[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            case 2:
                autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoder();
                DriveBase.leftMotor.setInverted(false);
                DriveBase.rightMotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[redLeft[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redLeft[1]].getInitialPose());
                currentStep++;
                break;
            case 4:
                // Run the second path of redLeft
                DriveBase.runTraj(trajectory[redLeft[1]], timer.get());
                if (timer.get() > trajectory[redLeft[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
        }
    }

    public static void DoRedMiddle() {
        switch (currentStep) {
            case 0:
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[redMiddle[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redMiddle[0]].getInitialPose());
                currentStep++;
                break;
            case 1:
                // Run the first path of redMiddle
                DriveBase.runTraj(trajectory[redMiddle[0]], timer.get());
                if (timer.get() > trajectory[redMiddle[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            case 2:
                autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.resetEncoder();
                DriveBase.leftMotor.setInverted(false);
                DriveBase.rightMotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[redMiddle[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redMiddle[1]].getInitialPose());
                currentStep++;
                break;
            case 4:
                // Run the second path of redMiddle
                DriveBase.runTraj(trajectory[redMiddle[1]], timer.get());
                if (timer.get() > trajectory[redMiddle[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);
        }
    }

    public static void DoRedRight() {
        switch (currentStep) {
            case 0:
                timer.reset();
                timer.start();
                DriveBase.odometry.resetPosition(trajectory[redRight[0]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redRight[0]].getInitialPose());
                currentStep++;
                break;
            case 1:
                // Run the first path of redRight
                DriveBase.runTraj(trajectory[redRight[0]], timer.get());
                if (timer.get() > trajectory[redRight[0]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            case 2:
                autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoder();
                DriveBase.leftMotor.setInverted(false);
                DriveBase.rightMotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[redRight[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redRight[1]].getInitialPose());
                currentStep++;
                break;
            case 4:
                // Run the second path of redRight
                DriveBase.runTraj(trajectory[redRight[1]], timer.get());
                if (timer.get() > trajectory[redRight[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftMotor.setInverted(true);
                DriveBase.rightMotor.setInverted(false);// need to test
        }
    }

    // Autonomous written with timer
    public static void DoLeftRightTimer() {
        double leftWheelVoltage = 0.5;
        double rightWheelVoltage = 0.5;
        if (timer.get() <= 1.5) {
            DriveBase.directControl(leftWheelVoltage, rightWheelVoltage);
        } else if (timer.get() > 1.5 && timer.get() <= 6) {
            // arm and intake
            autoArmControl(3, 2);
            Intake.solOn();
        } else if (timer.get() > 6 && timer.get() <= 10.5) {
            // arm
            autoArmControl(0, 1);
        } else if (timer.get() > 10.5 && timer.get() < 15) {
            DriveBase.directControl(-leftWheelVoltage, -rightWheelVoltage);
        }
    }

    public static void DoMiddleTimer() {
        double leftWheelVoltage = 0.5;
        double rightWheelVoltage = 0.5;
        if (timer.get() <= 1) {
            DriveBase.directControl(leftWheelVoltage, rightWheelVoltage);
        } else if (timer.get() > 1 && timer.get() <= 5.5) {
            autoArmControl(3, 2);
            Intake.solOn();
        } else if (timer.get() > 5.5 && timer.get() <= 10) {
            // arm
            autoArmControl(0, 0);
        } else if (timer.get() > 10 && timer.get() < 14.5) {
            DriveBase.directControl(-leftWheelVoltage, -rightWheelVoltage);
        } else {
            DriveBase.directControl(0, 0);
        }
    }

    public static void DoGoBackTimer() {
        double leftWheelVoltage = 0.8;
        double rightWheelVoltage = 0.8;
        if (timer.get() <= 1.3) {
            DriveBase.directControl(leftWheelVoltage, rightWheelVoltage);
        } else {
            DriveBase.directControl(0, 0);
        }
        DriveBase.putDashboard();
    }

    public static int autoArmControl(int modeLine, int modeArm) {
        switch (modeArm) {
            case 0: // the beginning position
                Arm.setArmSetpoint(68.5);
                break;
            case 1: // the second level
                Arm.setArmSetpoint(-10);
                break;
            case 2: // the third level
                Arm.setArmSetpoint(35.5);
                break;
            default:
                break;
        }

        switch (modeLine) {
            case 0: // the beginning position
                Arm.setLineSetpoint(0);
                break;
            case 2: // the second level
                Arm.setLineSetpoint(33.02);
                break;
            case 3: // the third level
                Arm.setLineSetpoint(86.15);
                break;
            default:
                break;
        }

        Arm.armControlLoop();
        // Arm.lineControlLoop();
        return 0;
    }

    public static void doMiddle(){
        double degree = DriveBase.getGyroDegree();
        lastTime = timer.getFPGATimestamp();
        if(degree > 0){
            if(degree >= 5){
                goChargeStation();
            }else{
                DriveBase.directControl(0.4, 0.4);
            }
        }else if(degree < 0){
           
            if(degree <= -5){
                goChargeStation();
            }else{
                DriveBase.directControl(0.4, 0.4);
            }
        }else{
            DriveBase.directControl(0.4, 0.4);
        }
    }

    public static void goChargeStation(){
        double Kp = ;
        double Kd = ;
        double goal = 0;
        double error = DriveBase.getGyroDegree()-goal;
        double limitDegree = ;
        double limitTime = timer.getFPGATimestamp()- lastTime;
        double m ; 
        double driveSpeed = Kp*error+Kd*;
    }
}
