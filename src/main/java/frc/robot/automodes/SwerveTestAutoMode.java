package frc.robot.automodes;

import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMultiPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.common.SwerveSpeedAngleAction;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.SwerveRobot2023Subsystem;
import frc.robot.subsystems.ARMSubsystem.ARMSubsystem;
import frc.robot.subsystems.GPMSubsystem.GPMSubsystem;
import frc.robot.subsystems.GrabberSubsystem.GrabberSubsystem;

public class SwerveTestAutoMode extends TestAutoMode {

    public SwerveTestAutoMode(SwerveRobot2023AutoModeController ctrl) throws Exception {
        super(ctrl, "Swerver-Test-Mode");

        SwerveRobot2023Subsystem robotsys = (SwerveRobot2023Subsystem) ctrl.getRobot().getRobotSubsystem();
        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) robotsys.getDB();
        MotorEncoderSubsystem armFirst = null ;
        MotorEncoderSubsystem armSecond = null ;

        GPMSubsystem gpm = robotsys.getGPM() ;
        ARMSubsystem arm = gpm.getARM() ;

        if (RobotBase.isSimulation()) {
            armFirst = arm.getFirst() ;
            armSecond = arm.getSecond() ;
        }

        double[] angles = new double[4];
        double[] powers = new double[4];

        switch (getTestNumber()) {
            case 0:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds)
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true) ;
                break;

            case 1:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 2:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds).  Since speed is given, the PID controller will try to
                // maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed")), true) ;
                break;

            case 3:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run  until the duration has expired.
                // Since speed is given, the PID controller will try to maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed"), getDouble("duration")), true) ;
                break ;                

            case 4:
                // Run the path follower against the path given
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), true), true);
                break ;

            case 5:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                angles[0] = getDouble("angle");
                angles[1] = getDouble("angle");
                angles[2] = getDouble("angle");
                angles[3] = getDouble("angle");
                powers[0] = getDouble("power");
                powers[1] = getDouble("power");
                powers[2] = getDouble("power");
                powers[3] = getDouble("power");
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, angles, powers, getDouble("duration")), true) ;
                break ;
                
            case 10:
                if (RobotBase.isSimulation()) {
                    addSubActionPair(armFirst, new MotorEncoderPowerAction(armFirst, getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 11:
                if (RobotBase.isSimulation()) {
                    addSubActionPair(armSecond, new MotorEncoderPowerAction(armSecond, getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 12:
                if (RobotBase.isSimulation()) {
                    double [] mtimes = { 4.0, 4.0, 4.0, 4.0, 4.0 } ;
                    double [] mpowers = { 0.1, 0.3, 0.5, 0.7, 0.9 } ;
                    addSubActionPair(armFirst, new MotorEncoderMultiPowerAction(armFirst, mtimes, mpowers), true) ; ;
                }
                break ;                

        }
    }
}
