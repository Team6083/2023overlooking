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
    private static final String LeftRightTimer = "LeftRightTimer";
    private static final String MiddleTimer = "MiddleTimer";
    private static final String GoBackTimer = "GoBackTimer";

    public static Timer timer = new Timer();

    public static SendableChooser<String> chooser;
    public static String autoSeclected;

    public static void init() {

        chooser = new SendableChooser<String>();

        putChooser();

        for (int i = 0; i < trajectoryAmount; i++) {
            try {
                // Importing PathWeaver JSON
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON[i]);
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("unable to open trajectory" + trajJSON[i] + "\n" + ex.getMessage(),
                        ex.getStackTrace());
            }

            var pose = trajectory[i].getInitialPose();

            DriveBase.setODOPose(pose);
        }
    }

    public static void start() {
        currentStep = 0;

        autoSeclected = chooser.getSelected();

        DriveBase.resetEncoderOn();
        DriveBase.resetGyro();
        DriveBase.resetPID();

        double autoDelayTime = SmartDashboard.getNumber("autoDelay", 0);

        Timer.delay(autoDelayTime);
        timer.reset();
        timer.start();
    }

    public static void loop() {

        DriveBase.updateODO();
        DriveBase.putDashboard();

        SmartDashboard.putNumber("AutoTimer", timer.get());
        SmartDashboard.putNumber("CurrentStep", currentStep);

        switch (autoSeclected) {
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
            case LeftRightTimer:
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
        chooser.setDefaultOption("DoNothing", DoNothing);
        chooser.addOption("BlueLeft", BlueLeft);
        chooser.addOption("BlueMiddle", BlueMiddle);
        chooser.addOption("BlueRight", BlueRight);
        chooser.addOption("RedLeft", RedLeft);
        chooser.addOption("RedMiddle", RedMiddle);
        chooser.addOption("RedRight", RedRight);

        chooser.addOption("LeftRightTimer", LeftRightTimer);
        chooser.addOption("MiddleTimer", MiddleTimer);
        chooser.addOption("GoBack", GoBackTimer);
        SmartDashboard.putData(chooser);
    }

    public static void DoBlueLeft() {
        switch (currentStep) {
            case 0:
                DriveBase.resetEncoderOff();
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
                Arm.autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                Arm.autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoderOn();
                DriveBase.resetEncoderOff();
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
                DriveBase.leftmotor.setInverted(true);
                DriveBase.rightmotor.setInverted(false);
        }
    }

    public static void DoBlueMiddle() {
        switch (currentStep) {
            case 0:
                DriveBase.resetEncoderOff();
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
                Arm.autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                Arm.autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.resetEncoderOn();
                DriveBase.resetEncoderOff();
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
                DriveBase.leftmotor.setInverted(true);
                DriveBase.rightmotor.setInverted(false);
        }
    }

    public static void DoBlueRight() {
        switch (currentStep) {
            case 0:
                DriveBase.resetEncoderOff();
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
                Arm.autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                Arm.autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoderOn();
                DriveBase.resetEncoderOff();
                DriveBase.leftmotor.setInverted(false);
                DriveBase.rightmotor.setInverted(true);
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
                DriveBase.leftmotor.setInverted(true);
                DriveBase.rightmotor.setInverted(false);
        }
    }

    public static void DoRedLeft() {
        switch (currentStep) {
            case 0:
                DriveBase.resetEncoderOff();
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
                Arm.autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                Arm.autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoderOn();
                DriveBase.resetEncoderOff();
                DriveBase.leftmotor.setInverted(false);
                DriveBase.rightmotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[redLeft[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redLeft[1]].getInitialPose());
                break;
            case 4:
                // Run the second path of redLeft
                DriveBase.runTraj(trajectory[redLeft[1]], timer.get());
                if (timer.get() > trajectory[redLeft[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftmotor.setInverted(true);
                DriveBase.rightmotor.setInverted(false);
        }
    }

    public static void DoRedMiddle() {
        switch (currentStep) {
            case 0:
                DriveBase.resetEncoderOff();
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
                Arm.autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                Arm.autoArmControl(0, 0);
                timer.reset();
                timer.start();
                DriveBase.resetEncoderOn();
                DriveBase.resetEncoderOff();
                DriveBase.leftmotor.setInverted(false);
                DriveBase.rightmotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[redMiddle[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redMiddle[1]].getInitialPose());
                break;
            case 4:
                // Run the second path of redMiddle
                DriveBase.runTraj(trajectory[redMiddle[1]], timer.get());
                if (timer.get() > trajectory[redMiddle[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftmotor.setInverted(true);
                DriveBase.rightmotor.setInverted(false);
        }
    }

    public static void DoRedRight() {
        switch (currentStep) {
            case 0:
                DriveBase.resetEncoderOff();
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
                Arm.autoArmControl(3, 2);
                Intake.solOn();
                currentStep++;
                break;
            case 3:
                // Turn around the arm
                Arm.autoArmControl(0, 1);
                timer.reset();
                timer.start();
                DriveBase.resetEncoderOn();
                DriveBase.resetEncoderOff();
                DriveBase.leftmotor.setInverted(false);
                DriveBase.rightmotor.setInverted(true);
                DriveBase.odometry.resetPosition(trajectory[redRight[1]].getInitialPose().getRotation(),
                        DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition()),
                        DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition()),
                        trajectory[redRight[1]].getInitialPose());
                break;
            case 4:
                // Run the second path of redRight
                DriveBase.runTraj(trajectory[redRight[1]], timer.get());
                if (timer.get() > trajectory[redRight[1]].getTotalTimeSeconds()) {
                    currentStep++;
                }
                break;
            default:
                DriveBase.leftmotor.setInverted(true);
                DriveBase.rightmotor.setInverted(false);// need to test
        }
    }

    // Autonomous written with timer
    public static void DoLeftRightTimer() {
        double leftV = 0.5;
        double rightV = 0.5;
        if (timer.get() <= 1.5) {
            DriveBase.directControl(leftV, rightV);
        } else if (timer.get() > 1.5 && timer.get() <= 6) {
            // arm and intake
            Arm.autoArmControl(3, 2);
            Intake.solOn();
        } else if (timer.get() > 6 && timer.get() <= 10.5) {
            // arm
            Arm.autoArmControl(0, 1);
        } else if (timer.get() > 10.5 && timer.get() < 15) {
            DriveBase.directControl(-leftV, -rightV);
        }
    }

    public static void DoMiddleTimer() {
        double leftV = 0.5;
        double rightV = 0.5;
        if (timer.get() <= 1) {
            DriveBase.directControl(leftV, rightV);
        } else if (timer.get() > 1 && timer.get() <= 5.5) {
            Arm.autoArmControl(3, 2);
            Intake.solOn();
        } else if (timer.get() > 5.5 && timer.get() <= 10) {
            // arm
            Arm.autoArmControl(0, 0);
        } else if (timer.get() > 10 && timer.get() < 14.5) {
            DriveBase.directControl(-leftV, -rightV);
        } else {
            DriveBase.directControl(0, 0);
        }
    }

    public static void DoGoBackTimer() {
        double leftV = 0.8;
        double rightV = 0.8;
        if (timer.get() <= 1.3) {
            DriveBase.directControl(leftV, rightV);
        } else {
            DriveBase.directControl(0, 0);
        }
        DriveBase.putDashboard();
    }
}
