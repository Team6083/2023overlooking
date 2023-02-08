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
import frc.robot.component.DriveBase;

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
    static Trajectory[] trajectory = new Trajectory[trajectoryAmount];

    public static Timer timer = new Timer();
    public static SendableChooser<String> chooser;
    public static String autoSeclected;

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
            case RedLeft:
                
        }
    }

    public static void BlueLeft(){
        switch(currentStep){
            case 0:
                    currentStep++;
                    DriveBase.resetEncoderOff();
                    timer.reset();
                    timer.start();
                    DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[0], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[0], timer.get());
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
                    DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[0], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[0], timer.get());
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
                    DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[0], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[0], timer.get());
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
                    DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[0], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[0], timer.get());
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
                    DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[0], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[0], timer.get());
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
                    DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    break;
            case 1: 
                    DriveBase.runTraj(trajectory[0], timer.get());
                    if(trajectory[0].getTotalTimeSeconds()>timer.get()){
                        currentStep++;
                        timer.reset();
                        timer.start();
                        DriveBase.resetEncoderOn();
                        DriveBase.resetEncoderOff();
                        DriveBase.odometry.resetPosition(trajectory[1].getInitialPose().getRotation()
                    , DriveBase.positionToDistanceMeter(DriveBase.leftMotor1.getSelectedSensorPosition())
                    , DriveBase.positionToDistanceMeter(DriveBase.rightMotor1.getSelectedSensorPosition())
                    ,trajectory[0].getInitialPose());
                    
                    }
                    break;
            case 2:
                    DriveBase.runTraj(trajectory[0], timer.get());
                    break;
                    
        }
    }
}