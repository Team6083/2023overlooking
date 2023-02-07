package frc.robot.component;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class Intake {
    private static Compressor com ;
    private static DoubleSolenoid sol;
    private static boolean sol_Forward = true;
    public static void init() {
    com = new Compressor(null);
    sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    }
    public static void teleop() {
    if(Robot.xbox.getYButton()){
        com.enableDigital();
    }else if(Robot.xbox.getAButton()){
        com.disable();
    }
    if(Robot.xbox.getBButtonPressed()){
        sol_Forward = !sol_Forward;
    }
    if (Robot.xbox.getBButton()) {
        if (sol_Forward== false) {
            sol.set(Value.kForward);
        } else {
            sol.set(Value.kReverse);
        }
    } else {
        sol.set(Value.kOff);
    }
    }
}
