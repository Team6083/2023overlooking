package frc.robot.component;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class Intake {
    private static Compressor com ;
    private static DoubleSolenoid sol1;
    private static DoubleSolenoid sol2;
    private static boolean sol_Forward = true;
    public static void init() {
    com = new Compressor(null);
    sol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
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
            sol1.set(Value.kForward);
        } else {
            sol1.set(Value.kReverse);
        }
    } else {
        sol1.set(Value.kOff);
    }
    }

    public static void solOn(){
        sol1.set(Value.kForward);
    }

    public static void solOff(){
        sol1.set(Value.kOff);
    }
}
