package frc.robot.component;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class Intake {
    private static Compressor com;
    private static DoubleSolenoid sol;
    private static boolean solForward = false;

    public static void init() {
        com = new Compressor(PneumaticsModuleType.CTREPCM);
        sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        com.enableDigital();
    }

    public static void teleop() {
        if (Robot.mainController.getYButtonPressed()) {
            solForward = !solForward;
        }
        if (solForward) {
            sol.set(Value.kForward);
        } else if (!solForward) {
            sol.set(Value.kReverse);
        } else {
            sol.set(Value.kOff);
        }
    }

    public static void solOn() {
        sol.set(Value.kForward);
    }

    public static void solOff() {
        sol.set(Value.kOff);
    }
}
