package frc.robot.subsystems.GrabberSubsystem;

import org.xero1425.base.motors.MotorController;
import org.xero1425.base.pneumatics.XeroDoubleSolenoid;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class GrabberSubsystem extends Subsystem {

    final static String SubsystemName = "grabber" ;

    private XeroDoubleSolenoid solenoid_ ;
    private MotorController spinner_ ;

    public GrabberSubsystem(Subsystem parent) throws BadParameterTypeException, MissingParameterException {
        super(parent, SubsystemName) ;

        int module = getSettingsValue("hw:solenoid:module").getInteger() ;
        int forward = getSettingsValue("hw:solenoid:forward").getInteger() ;
        int reverse = getSettingsValue("hw:solenoid:reverse").getInteger() ;

        solenoid_ = new XeroDoubleSolenoid(parent.getRobot(), module, forward, reverse) ;

        String motorname = "subsystems:" + SubsystemName + ":hw:spinner:motor" ;
        spinner_ = getRobot().getMotorFactory().createMotor("intake-spinner", motorname);
    }

    public void open() {
        solenoid_.set(Value.kForward) ;
    }

    public void close() {
        solenoid_.set(Value.kReverse) ;
    }

    public void setSpinnerPower(double power) {
        try {
            spinner_.set(power) ;
        }
        catch(Exception ex) {
        }
    }
}
