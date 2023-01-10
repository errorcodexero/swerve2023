// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SimArgs;
import org.xero1425.misc.XeroPathType;

import frc.robot.automodes.SwerveRobot2023AutoModeController;
import frc.robot.subsystems.SwerveRobot2023Subsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class SwerveRobot2023 extends XeroRobot {

  public SwerveRobot2023() {
    super(0.02) ;
  }
  
  public String getName() {
    return "SwerveRobot2023";
  }

  public String getSimulationFileName() {
      String ret = SimArgs.InputFileName;
      if (ret != null)
          return ret;

      return "automode";
  }

  protected void hardwareInit() throws Exception {
      SwerveRobot2023Subsystem robot = new SwerveRobot2023Subsystem(this);
      setRobotSubsystem(robot);
  }

  @Override
  protected AutoController createAutoController() throws MissingParameterException, BadParameterTypeException {
    AutoController ctrl;

    try {
        ctrl = new SwerveRobot2023AutoModeController(this);
    } catch (Exception ex) {
        ctrl = null;
    }

    return ctrl;
  }

  protected XeroPathType getPathType() {
    return XeroPathType.SwerveHolonomic ;
  }
}
