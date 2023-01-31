package frc.robot.component;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class Intake {
    private static Compressor com ;
    private static DoubleSolenoid solleft;
    private static DoubleSolenoid solright;
    private static boolean solleft_Forward = true;
    private static boolean solright_Forward = true;
    public static void init() {
    com = new Compressor(null);
    solleft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    solright = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    }
    public static void teleop() {
    if(Robot.xbox.getYButton()){
        com.enableDigital();
    }else if(Robot.xbox.getAButton()){
        com.disable();
    }
    if(Robot.xbox.getBButtonPressed()){
        solleft_Forward = !solleft_Forward;
    }
    if (Robot.xbox.getBButton()) {
        if (solleft_Forward== false) {
            solleft.set(Value.kForward);
        } else {
            solleft.set(Value.kReverse);
        }
    } else {
        solleft.set(Value.kOff);
    }
    if(Robot.xbox.getXButtonPressed()){
        solright_Forward = !solright_Forward;
    }
    if(Robot.xbox.getXButton()){
        if(solright_Forward==false){
            solright.set(Value.kForward);
        }else{
            solright.set(Value.kReverse);
        }
    }else{
        solright.set(Value.kOff);
    }

    }
}
