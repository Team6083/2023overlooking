package frc.robot.component;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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

    // Port
    private static final int leftMotorID1 = 13;// MotorController ID
    private static final int leftMotorID2 = 14;
    private static final int rightMotorID1 = 11;
    private static final int rightMotorID2 = 12;

    // Basis divebase
    public static WPI_TalonSRX leftMotor1;
    public static WPI_VictorSPX leftMotor2;
    public static WPI_TalonSRX rightMotor1;
    public static WPI_VictorSPX rightMotor2;

    public static MotorControllerGroup leftMotor;
    public static MotorControllerGroup rightMotor;
    public static DifferentialDrive drive;// Used to simplify drivebase program

    // Sensor
    public static AHRS gyro;

    public static DifferentialDriveOdometry odometry;
    protected static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.53);
    protected static RamseteController ramseteController = new RamseteController();

    // Feedforward Controller
    protected static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.63228, 2.1943);

    protected static Field2d field = new Field2d();
    protected static Field2d trajField = new Field2d();

    // For PID
    private static double kP = 2.063;
    private static double kI = 0;
    private static double kD = 0;

    public static PIDController leftPID = new PIDController(kP, kI, kD);
    public static PIDController rightPID = new PIDController(kP, kI, kD);

    // Set the pulse of the encoder
    private static final double encoderPulse = 4096;

    private static final double gearRatio = 7.31;

    private static double leftMotorVolt;
    private static double rightMotorVolt;

    private static double leftMotorSpeedInput;
    private static double rightMotorSpeedInput;

    private static double leftWheelSpeed;
    private static double rightWheelSpeed;

    public static void init() {
        leftMotor1 = new WPI_TalonSRX(leftMotorID1);
        leftMotor2 = new WPI_VictorSPX(leftMotorID2);
        rightMotor1 = new WPI_TalonSRX(rightMotorID1);
        rightMotor2 = new WPI_VictorSPX(rightMotorID2);

        leftMotor = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotor = new MotorControllerGroup(rightMotor1, rightMotor2);
        leftMotor.setInverted(true);
        leftMotor1.setSensorPhase(true);
        drive = new DifferentialDrive(leftMotor, rightMotor);

        // Reset encoder
        resetEncoder();

        // Define gyro ID
        gyro = new AHRS(SPI.Port.kMXP);

        // Put path and status on field from PathWeaver on SmartDashboard
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0),
                positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()),
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));

        SmartDashboard.putData("field", field);
        SmartDashboard.putData("trajField", trajField);

        putDashboard();
    }

    // Normal drivebase
    public static void teleop() {

        double speedOutputRatio = (Robot.mainController.getLeftBumper() || Robot.mainController.getRightBumper()) ? 1 : 0.8;
        leftMotorSpeedInput = Robot.mainController.getLeftY() * speedOutputRatio;
        rightMotorSpeedInput = Robot.mainController.getRightY() * speedOutputRatio;

        drive.tankDrive(leftMotorSpeedInput, rightMotorSpeedInput);
        putDashboard();
    }

    public static void directControl(double leftMotorInput, double rightMotorInput) {
        drive.tankDrive(leftMotorInput, rightMotorInput);
    }

    public static void runTraj(Trajectory trajectory, double timeInsec) {

        // Set the goal of the robot in that second
        Trajectory.State goal = trajectory.sample(timeInsec);
        trajField.setRobotPose(goal.poseMeters);

        var currentPose = odometry.getPoseMeters();

        var chaspeed = ramseteController.calculate(currentPose, goal);

        // Convert chassis speed to wheel speed
        var wheelSpeeds = kinematics.toWheelSpeeds(chaspeed); // Left and right speed
        leftWheelSpeed = wheelSpeeds.leftMetersPerSecond; // Catch speed from wheelSpeed(with ctrl + left mice)
        rightWheelSpeed = wheelSpeeds.rightMetersPerSecond;

        SmartDashboard.putNumber("leftWheelSpeed", leftWheelSpeed);
        SmartDashboard.putNumber("rightWheelSpeed", rightWheelSpeed);

        leftPID.setSetpoint(leftWheelSpeed);
        rightPID.setSetpoint(rightWheelSpeed);

        // To make the number of the encoder become the motor's volt
        leftMotorVolt = leftPID.calculate(
                positionToDistanceMeter(leftMotor1.getSelectedSensorVelocity()),
                leftMotorSpeedInput)
                + feedforward.calculate(leftWheelSpeed);
        rightMotorVolt = rightPID.calculate(
                positionToDistanceMeter(rightMotor1.getSelectedSensorVelocity()),
                rightMotorSpeedInput)
                + feedforward.calculate(rightWheelSpeed);

        leftMotor.setVoltage(leftMotorVolt);
        rightMotor.setVoltage(rightMotorVolt);
        drive.feed();

        // put the distances between the target and current position onto Dashboard
        SmartDashboard.putNumber("left_volt", leftMotorVolt);
        SmartDashboard.putNumber("right_volt", rightMotorVolt);

        SmartDashboard.putNumber("errorPosX", currentPose.minus(goal.poseMeters).getX());
        SmartDashboard.putNumber("errorPosY", currentPose.minus(goal.poseMeters).getY());

    }

    public static void updateODO() {
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());

        var leftPos = positionToDistanceMeter(leftMotor1.getSelectedSensorPosition());
        odometry.update(gyroAngle, leftPos,
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()));
        field.setRobotPose(odometry.getPoseMeters());

        SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("heading", odometry.getPoseMeters().getRotation().getDegrees());

        SmartDashboard.putNumber("leftPos", leftPos);

        // number on Dashboard can be adjusted
        kP = SmartDashboard.getNumber("kP", kP);
        kI = SmartDashboard.getNumber("kI", kI);
        kD = SmartDashboard.getNumber("kD", kD);

        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        drive.feed();
    }

    // To set the position and rotation of the robot
    public static void setODOPose(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(), positionToDistanceMeter(leftMotor1.getSelectedSensorPosition()),
                positionToDistanceMeter(rightMotor1.getSelectedSensorPosition()), pose);
        field.setRobotPose(odometry.getPoseMeters());
    }

    // Input the position of the encoder
    public static void putDashboard() {
        SmartDashboard.putNumber("leftEncoder", leftMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightEncoder", rightMotor1.getSelectedSensorPosition());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putNumber("leftController_speed", leftMotorSpeedInput);
        SmartDashboard.putNumber("rightController_speed", rightMotorSpeedInput);
        SmartDashboard.putNumber("left_wheel_speed", leftWheelSpeed);
        SmartDashboard.putNumber("right_wheel_speed", rightWheelSpeed);
    }

    // Calculate the distance(meter)
    public static double positionToDistanceMeter(double position) {
        double sensorRate = position / encoderPulse;
        double wheelRate = sensorRate * 0.5;

        double positionMeter = 2 * Math.PI * Units.inchesToMeters(6) * wheelRate;
        return positionMeter;
    }

    // Here comes some mode to set up or update
    public static void resetEncoder() {
        leftMotor1.setSelectedSensorPosition(0);
        rightMotor1.setSelectedSensorPosition(0);
    }

    public static void resetGyro() {
        gyro.reset();
    }

    public static void resetPID() {
        leftPID.reset();
        rightPID.reset();
    }

    public static double getGyroDegree() {
        return gyro.getPitch();
    }

    // public static void driveControl(){
    // double x = Robot.mainController.getLeftX();
    // double y = Robot.mainController.getLeftY();
    // directControl(0.8*(y+x), 0.8*(y-x));
    // }
}
