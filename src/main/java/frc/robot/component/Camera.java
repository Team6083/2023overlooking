package frc.robot.component;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Camera {
    private static UsbCamera camera;

    public static void init() {
        camera = CameraServer.startAutomaticCapture(0);
        camera.setFPS(24);
    }

}
