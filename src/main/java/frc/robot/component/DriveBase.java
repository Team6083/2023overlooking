package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.System.NewAutoEngine;

public class DriveBase {
    
    //port
    private static final int Lm1 = 1;
    private static final int Lm2 = 2;
    private static final int Rm1 = 3;
    private static final int Rm2 = 4;

    //basis divebase
    public static WPI_TalonSRX leftMotor1;//define motor
    public static WPI_TalonSRX leftMotor2;
    public static WPI_TalonSRX rightMotor1;
    public static WPI_TalonSRX rightMotor2;
    public static MotorControllerGroup leftmotor;
    public static MotorControllerGroup rightmotor;
    public static DifferentialDrive drive;

    //gyro
    public static AHRS gyro;

    //for PID
    public static double kP = 0.13;
    public static double kI = 0;
    public static double kD = 0;

    protected static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.7, 0.1);
    //PIDController
    public static PIDController leftPID = new PIDController(kP, kI, kD);
    public static PIDController rightPID = new PIDController(kP, kI, kD);

    protected static RamseteController ramseteController = new RamseteController();
    protected static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.63);

    public static DifferentialDriveOdometry odometry;
    protected static Field2d field = new Field2d();
    protected static Field2d trajField = new Field2d();

    public static final double encoderPulse = 4096;
    public static final double gearing = 10.71;

    public static void init() {
        leftMotor1 = new WPI_TalonSRX(Lm1);
        leftMotor2 = new WPI_TalonSRX(Lm2);
        rightMotor1 = new WPI_TalonSRX(Rm1);
        rightMotor2 = new WPI_TalonSRX(Rm2);

        leftmotor = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightmotor = new MotorControllerGroup(rightMotor1, rightMotor2);
        rightmotor.setInverted(true);
        drive = new DifferentialDrive(leftmotor, rightmotor);

        //reset encoder
        leftMotor1.configClearPositionOnQuadIdx(true, 10);
        rightMotor1.configClearPositionOnQuadIdx(true, 10);

        //define gyro ID
        gyro = new AHRS(SPI.Port.kMXP);

        
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), positionToDistanceMeter(leftMotor1.getSelectedSensorPosition())
                                                                                   , positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));
        SmartDashboard.putData("field",field);
        SmartDashboard.putData("trajField",trajField);
    }

    public static void teloop(){
        putDashboard();

        double leftV = Robot.xbox.getLeftY()*0.9;
        double rightV = Robot.xbox.getRightY()*0.9;

        drive.tankDrive(leftV, rightV);
    }
    public static void resetEncoderOn(){
        leftMotor1.configClearPositionOnQuadIdx(true, 10);
        rightMotor1.configClearPositionOnQuadIdx(true, 10);
    }

    public static void resetEncoderOff(){
        leftMotor1.configClearPositionOnQuadIdx(false, 10);
        rightMotor1.configClearPositionOnQuadIdx(false, 10);
    }

    public static void resetGyro(){
        gyro.reset();
    }

    public static void resetPID(){
        leftPID.reset();
        rightPID.reset();
    }

    public static double positionToDistanceMeter(double position){
        double sensorRate = position/encoderPulse;
        double wheelRate = sensorRate/gearing;
        double positionMeter = 2*Math.PI*Units.inchesToMeters(6)*wheelRate;
        return positionMeter;
    }

    public static void track(double speed, double rotation, boolean input){
        drive.arcadeDrive(speed, rotation,input);
    }

    public static void directControl(double left, double right){
        drive.tankDrive(left, right);
    }

    public static void updateODO(){
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        odometry.update(gyroAngle, positionToDistanceMeter(leftMotor1.getSelectedSensorPosition())
                                 , positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());

        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);

        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        drive.feed();
    }

    public static void setODOPose(Pose2d pose){
        odometry.resetPosition(pose.getRotation(), positionToDistanceMeter(leftMotor1.getSelectedSensorPosition())
                                                 , positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()), pose);
        field.setRobotPose(odometry.getPoseMeters()); 
    }

    public static void putDashboard(){
        SmartDashboard.putNumber("leftEncoder", leftMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightEncoder", rightMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
    }

    public static void runTraj(Trajectory trajectory, double timeInsec){
        Trajectory.State goal = trajectory.sample(timeInsec);
        trajField.setRobotPose(goal.poseMeters);

        var currentPose = odometry.getPoseMeters();

        var chaspeed = ramseteController.calculate(currentPose, goal);

        var wheelSpeeds = kinematics.toWheelSpeeds(chaspeed);
        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;

        double leftVolt = leftPID.calculate(positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()/NewAutoEngine.timer.get()), left) + feedforward.calculate(left);
        double rightVolt = leftPID.calculate(positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()/NewAutoEngine.timer.get()), left) + feedforward.calculate(right);
    
        leftmotor.setVoltage(leftVolt);
        rightmotor.setVoltage(rightVolt);
        drive.feed();

        SmartDashboard.putNumber("leftVolt", leftVolt);
        SmartDashboard.putNumber("rightVolt", rightVolt);
        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);
        SmartDashboard.putNumber("left_error", leftPID.getPositionError());
        SmartDashboard.putNumber("right_error", rightPID.getPositionError());
        SmartDashboard.putNumber("errorPosX", currentPose.minus(goal.poseMeters).getX());
        SmartDashboard.putNumber("errorPosY", currentPose.minus(goal.poseMeters).getY());
    }

}
