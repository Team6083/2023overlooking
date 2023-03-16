// package frc.robot.component;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;

// public class Limelight {
//     private static double tx;
//     private static double tv;
//     private static NetworkTable table;
//     private static PIDController pid;
//     protected static boolean Limelight_SwitchLeft = false;
//     protected static boolean Limelight_SwitchRight = false;
//     private static double hDriveSpeed;
//     private static final double hDrive_MaxSpeed = 0.8; // to be confirmed
//     private static final double DxNumber = Math.toDegrees(Math.atan2(12, 88)); // to be confirmed

//     public static void init() {
//         pid = new PIDController(0.04, 0.001, 0.0001); // to be confirmed
//         setLEDMode(1);
//         setCamMode(1);
//     }

//     public static void teleop() {
//         if (Robot.xbox.getXButtonPressed()) { // buttons to be confirmed
//             Limelight_SwitchLeft = !Limelight_SwitchLeft;
//             if (Limelight_SwitchLeft) {
//                 Limelight_SwitchRight = false;
//             }
//         }
//         if (Robot.xbox.getYButtonPressed()) { // buttons to be confirmed
//             Limelight_SwitchRight = !Limelight_SwitchRight;
//             if (Limelight_SwitchRight) {
//                 Limelight_SwitchLeft = false;
//             }
//         }
//         if (Limelight_SwitchLeft || Limelight_SwitchRight) {
//             setLEDMode(0);
//             setCamMode(0);
//             aiming();
//         } else {
//             setLEDMode(1);
//             setCamMode(1);
//         }
//         putDashboard();
//     }

//     public static void aiming() {
//         table = NetworkTableInstance.getDefault().getTable("limelight");
//         tx = table.getEntry("tx").getDouble(0) - DxNumber; // Dxnumber: Horizontal Offset From Limelight To the Arm
//         tv = table.getEntry("tv").getDouble(0);
//         hDriveSpeed = pid.calculate(tx, 0.0);
//         if (tv > 0) {
//             if (hDriveSpeed > hDrive_MaxSpeed) {
//                 hDriveSpeed = hDrive_MaxSpeed;
//             } else if (hDriveSpeed < -hDrive_MaxSpeed) {
//                 hDriveSpeed = -hDrive_MaxSpeed;
//             }
//             DriveBase.middleMotor.set(hDriveSpeed);
//         } else {
//             finding();
//         }

//     }

//     public static void finding() {
//         if (Limelight_SwitchLeft) {
//             DriveBase.middleMotor.set(0.8);
//         }
//         if (Limelight_SwitchRight) {
//             DriveBase.middleMotor.set(-0.8);
//         }

//     }

//     public static void putDashboard() {
//         SmartDashboard.putBoolean("left limelight switch", Limelight_SwitchLeft);
//         SmartDashboard.putBoolean("right limelight switch", Limelight_SwitchRight);
//     }

//     /**
//      * 0 use the LED Mode set in the current pipeline
//      * 1 force off
//      * 2 force blink
//      * 3 force on
//      * 
//      * @param ModeNumber use to set LED mode
//      */

//     public static void setLEDMode(int ModeNumber) {
//         NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ModeNumber);
//     }

//     /**
//      * 0 Vision processor
//      * 1 Driver Camera (Increases exposure, disables vision processing)
//      *
//      * @param CamNumber use to set Cam mode
//      */

//     public static void setCamMode(int CamNumber) {
//         NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(CamNumber);
//     }

//     public static void disabledInit() {
//         setLEDMode(1);
//         setCamMode(1);
//     }
// }
