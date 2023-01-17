package frc.robot.subsystems.GPMSubsystem;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.Subsystem;

import frc.robot.subsystems.ARMSubsystem.ARMSubsystem;
import frc.robot.subsystems.GrabberSubsystem.GrabberSubsystem;

public class GPMSubsystem extends Subsystem {

    static final private String SubsystemName = "GPMSubsystem" ;

    private ARMSubsystem arm_ ;
    private GrabberSubsystem grabber_ ;

    public GPMSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName) ;

        if (XeroRobot.isSimulation()) {

            arm_ = new ARMSubsystem(this) ;
            addChild(arm_) ;

            grabber_ = new GrabberSubsystem(this) ;
            addChild(grabber_) ;
        }
    }

    public ARMSubsystem getARM() {
        return arm_ ;
    }

    public GrabberSubsystem getGrabber() {
        return grabber_ ;
    }
}