package frc.robot.subsystems.ARMSubsystem;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class ARMSubsystem extends Subsystem {
    private final static String SubsystemName = "arm" ;

    private MotorEncoderSubsystem first_ ;
    private MotorEncoderSubsystem second_ ;

    public ARMSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName) ;

        first_ = new MotorEncoderSubsystem(this, SubsystemName + "-first", true) ;
        addChild(first_) ;

        second_ = new MotorEncoderSubsystem(this, SubsystemName + "-second", true) ;
        addChild(second_) ;
    }

    public MotorEncoderSubsystem getFirst() {
        return first_ ;
    }

    public MotorEncoderSubsystem getSecond() {
        return second_ ;
    }
}
