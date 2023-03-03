package frc.robot.component;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensor {
    public static Rev2mDistanceSensor distSens;
    
    public static void init(){
        distSens = new Rev2mDistanceSensor(Port.kOnboard);
        distSens.setAutomaticMode(true);
    }
    
}
