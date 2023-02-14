package frc.robot.component;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Camera {
    private static UsbCamera camera;
    private static void init(){
        camera = CameraServer.startAutomaticCapture();
        camera.setFPS(24);
    }

}
