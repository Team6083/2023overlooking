package frc.robot.component;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor {
    public static Rev2mDistanceSensor distSens;

    public static void init() {
        distSens = new Rev2mDistanceSensor(Port.kOnboard);
        distSens.setAutomaticMode(true);
    }

    public static void teleop() {
        boolean correct = distSens.GetRange() * 2.54>15 && distSens.GetRange() * 2.54<19;
        SmartDashboard.putNumber("dis", distSens.GetRange() * 2.54);
        SmartDashboard.putBoolean("disCorrect", correct);
    }

    public static double getDistence(){
        return distSens.GetRange()*2.54;
    }
}
