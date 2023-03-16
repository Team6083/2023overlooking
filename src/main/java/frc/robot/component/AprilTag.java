// package frc.robot.component;

// import com.ctre.phoenix.sensors.Pigeon2;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;
// import frc.robot.component.DriveBase;
// import frc.robot.System.NewAutoEngine;

// public class AprilTag {
//     // distance estimating
//     public static final double dLOW_CUBE = 0.5;
//     public static final double dMIDDLE_CUBE = 0.7;
//     public static final double dHIGH_CUBE = 0.9;
//     public static final double rangeAreaMethod = 0.5;
//     public static double setpoint;

//     public static double kP;
//     public static double kI;
//     public static double kD;

//     public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//     public static NetworkTableEntry tx = table.getEntry("tx");
//     public static NetworkTableEntry ty = table.getEntry("ty");
//     public static NetworkTableEntry ta = table.getEntry("ta");
//     public static NetworkTableEntry tv = table.getEntry("tv");

//     public static PIDController pid_Mid;
//     public static PIDController pid_H;
//     public static PIDController pid_M;
//     public static PIDController pid_L;

//     public static void init() {
//         // read values periodically
//         double x = tx.getDouble(0.0);
//         double y = ty.getDouble(0.0);
//         double area = ta.getDouble(0.0);
//         double hasTarget = tv.getDouble(0);// 0 or 1
//         double pipeLineResult = NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe")
//                 .getDouble(0);
//         // double targetID =
//         // NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

//         // example code
//         if (area >= rangeAreaMethod) {
//             NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);// better
//             NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);// change pipeline
//                                                                                                       // to 1
//             double targetID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
//         } else if (area <= rangeAreaMethod) {
//             NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
//             NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
//         }

//         // post to smart dashboard periodically
//         SmartDashboard.putNumber("LimelightX", x);
//         SmartDashboard.putNumber("LimelightY", y);
//         SmartDashboard.putNumber("LimelightArea", area);
//         SmartDashboard.putNumber("Target?", hasTarget);
//         SmartDashboard.putNumber("Pipeline", pipeLineResult);// pipeline that's curretnly using
//         // SmartDashboard.putNumber("Target ID", targetID);

//         NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

//     }

//     public static void teleop () {// value caution
//         double targetOffsetAngle_Vertical = ty.getDouble(0.0);

//         // how many degrees back is your limelight rotated from perfectly vertical?
//         double limelightMountAngleDegrees = 21.0;

//         // distance from the centre of the Limelight lens to the floor
//         double limelightLensHeightInches = 20.0;

//         // distance from the target to the floor
//         double goalHeightInches = 60.0;

//         double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
//         double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//         // calculate distance
//         double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
//                 / Math.sin(angleToGoalDegrees + targetOffsetAngle_Vertical);// (goalHeightInches -
//                                                                             // limelightLensHeightInches)/
//                                                                             // Math.tan(angleToGoalRadians);
//         double distance2d = (goalHeightInches - limelightLensHeightInches)
//                 / Math.tan(angleToGoalDegrees + targetOffsetAngle_Vertical);

//         // target pose
//         double[] target_pose = NetworkTableInstance.getDefault().getTable("limelight")
//                 .getEntry("<targetpose_robotspace>")
//                 .getDoubleArray(new double[6]);
//         double t_X = target_pose[0];
//         double t_Y = target_pose[1];
//         double t_Z = target_pose[2];
//         double t_roll = target_pose[3];
//         double t_pitch = target_pose[4];
//         double t_yaw = target_pose[5];
//         double t_delay = target_pose[6];

//         // bot pose
//         double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<botpose>")
//                 .getDoubleArray(new double[6]);
//         SmartDashboard.putNumberArray("pulu_bot_Pose", botpose);
//         double bot_X = botpose[0];
//         double bot_Y = botpose[1];
//         double bot_Z = botpose[2];
//         double bot_roll = botpose[3];
//         double bot_pitch = botpose[4];
//         double bot_yaw = botpose[5];
//         double bot_delay = botpose[6];

//         // bot pose target space
//         double[] b_poseT = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<botpose_targetspace>")
//                 .getDoubleArray(new double[6]);
//         double b_X = b_poseT[0];
//         double b_Y = b_poseT[1];
//         double b_Z = b_poseT[2];
//         double b_roll = b_poseT[3];
//         double b_pitch = b_poseT[4];
//         double b_yaw = b_poseT[5];
//         double b_delay = b_poseT[6];
//         // double[] error =
//         // NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new
//         // double[6]);

//         // measurement calculating
//         //double measurement = b_X;
//         //return b_X;
//         double measurement = b_X;

//         if (Robot.xbox.getXButton()) {
//             // distance from bot to target
//             pid_Mid = new PIDController(kP, kI, kD);
//             //double test_speed;
//             pid_Mid.calculate(measurement);
//     }
//         if (Robot.xbox.getYButton()){
//             //forward/back
//             pid_H = new PIDController(kP, kI, kD);
//             pid_H.calculate(measurement, dHIGH_CUBE);
//         }

//         if (Robot.xbox.getBButton()){
//             pid_M = new PIDController(kP, kI, kD);
//             pid_M.calculate(measurement, dMIDDLE_CUBE);
//         }

//         if (Robot.xbox.getXButton()){
//             pid_L = new PIDController(kP, kI, kD);
//             pid_L.calculate(measurement, dLOW_CUBE);
//         }
// }

// }
