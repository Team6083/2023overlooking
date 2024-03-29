// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.System.NewAutoEngine;
import frc.robot.component.Arm;
import frc.robot.component.Camera;
import frc.robot.component.DriveBase;
import frc.robot.component.Intake;
import frc.robot.component.Light;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static XboxController mainController;
  public static XboxController viceController;

  public static PowerDistribution pd;

  public static Arm arm;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    pd = new PowerDistribution(1, ModuleType.kCTRE);
    SmartDashboard.putData(pd);

    mainController = new XboxController(0);
    viceController = new XboxController(1);
    DriveBase.init();
    Intake.init();
    Camera.init();
    Light.init();

    arm = new Arm();
    NewAutoEngine.init();
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    NewAutoEngine.start();
  }

  @Override
  public void autonomousPeriodic() {
    NewAutoEngine.loop();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    DriveBase.teleop();
    Intake.teleop();
    Light.teleop();
    arm.teleop(mainController, viceController);
    SmartDashboard.putNumber("pdp_0_current", pd.getCurrent(0));
  }

  @Override
  public void disabledInit() {
    Light.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
