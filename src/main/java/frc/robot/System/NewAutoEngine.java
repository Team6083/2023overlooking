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
    static String[] trajJSON = {};
    private static final String DoNothing = "DoNothing";
    private static final String BlueLeft = "BlueLeft";
    private static final String BlueMiddle = "BlueMiddle";
    private static final String BlueRight = "BlueRight";
    private static final String RedLeft = "RedLeft";
    private static final String RedMiddle = "RedMiddle";
    private static final String RedRight = "RedRight";
    private static final int[] blueLeft = {0,1};
    private static final int[] blueMiddle = {2,3};
    private static final int[] blueRight = {4,5};
    private static final int[] redLeft = {6,7};
    private static final int[] redMiddle = {8,9};
    private static final int[] redRight = {10,11};
    static Trajectory[] trajectory = new Trajectory[trajectoryAmount];

    public static Timer timer = new Timer();
    public static SendableChooser<String> chooser;
    public static String autoSeclected;
    private static boolean k = true;

    public static void init() {
        chooser = new SendableChooser<String>();
        putChooser();
        for (int i = 0; i < trajectoryAmount; i++){
            try{
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajJSON[i]);
                trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch(IOException ex) {
                DriverStation.reportError("unable to open trajectory" + trajJSON[i] + "\n" + ex.getMessage(), ex.getStackTrace());
            }

            var pose = trajectory[i].getInitialPose();

            DriveBase.setODOPose(pose);
        }
    }
    public static void start(){
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
    protected static void putChooser(){
        chooser.setDefaultOption("DoNothing", DoNothing);
        chooser.setDefaultOption("BlueLeft", BlueLeft);
        chooser.setDefaultOption("BlueMiddle", BlueMiddle);
        chooser.setDefaultOption("BlueRight", BlueRight);
        chooser.setDefaultOption("RedLeft", RedLeft);
        chooser.setDefaultOption("RedMiddle", RedMiddle);
        chooser.setDefaultOption("RedRight", RedRight);
        SmartDashboard.putData(chooser);
    }
    public static void loop(){
        DriveBase.updateODO();
        DriveBase.putDashboard();
        SmartDashboard.putNumber("AutoTimer", timer.get());
        SmartDashboard.putNumber("CurrentStep", currentStep);
        switch(autoSeclected){
            case DoNothing:
                DriveBase.directControl(0, 0);
                break;
            case BlueLeft:
                BlueLeft();
                break;
            case BlueMiddle:
                BlueMiddle();
                break;
            case BlueRight:
                BlueRight();
                break;
            case RedLeft:
                RedLeft();
                break;
            case RedMiddle:
                RedMiddle();
                break;
            case RedRight:
                RedRight();
                break;
        }
    }

    public static void BlueLeft(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[blueLeft[0]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[blueLeft[0]].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[blueLeft[0]], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()<timer.get()){
                        currentStep++;
                    }
                    break;
            case 2:
                    if(Arm.autoAccessDegree()>31.5){
                        Arm.autoArm(0);
                        if(k){//encoder position of the 
                            Arm.autoVic(0);
                            Intake.solOn();
                            currentStep++;
                        }else{
                            Arm.autoVic(-0.9);
                        }
                    }else{
                        Arm.autoArm(0.85);
                    }
                    break;
            case 3:
                    if(k){//k = degree
                        Arm.autoArm(0);
                        Arm.autoVic(0);
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.leftMotor1.setInverted(true);
                        DriveBase.rightMotor1.setInverted(false);
                        DriveBase.odometry.resetPosition(trajectory[blueLeft[1]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[blueLeft[1]].getInitialPose());
                    }else{
                        Arm.autoArm(0.8);
                        Arm.autoVic(0.9);
                    }
                    break;
            case 4:
                    DriveBase.runTraj(trajectory[blueLeft[1]], timer.get());
                    if(trajectory[blueLeft[1]].getTotalTimeSeconds()<timer.get()){
                        currentStep++;
                    }
                    break;
            case 5:
                    Intake.solOff();
                    break;
                    
        }
    }

    public static void BlueMiddle(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[blueMiddle[0]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[blueMiddle[0]].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[blueMiddle[0]], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[blueMiddle[1]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[blueMiddle[1]].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[blueMiddle[1]], timer.get());
                    break;
                    
        }
    }

    public static void BlueRight(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[blueRight[0]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[blueRight[0]].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[blueRight[0]], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[blueRight[1]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[blueRight[1]].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[blueRight[0]], timer.get());
                    break;
                    
        }
    }
    public static void RedLeft(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[redLeft[0]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[redLeft[0]].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[redLeft[0]], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[redLeft[1]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[redLeft[1]].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[redLeft[1]], timer.get());
                    break;
                    
        }
    }
    public static void RedMiddle(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[redMiddle[0]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[redMiddle[0]].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[redMiddle[0]], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[redMiddle[1]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[redMiddle[1]].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[redMiddle[1]], timer.get());
                    break;
                    
        }
    }
    public static void RedRight(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[redRight[0]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[redRight[0]].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[redRight[0]], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[redRight[1]].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[redRight[1]].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[redRight[1]], timer.get());
                    break;
                    
        }
    }
}