package frc.robot.subsystems;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.OISubsystem;

public class SwerveRobot2023OISubsystem extends OISubsystem {
    private final static String SubsystemName = "swerve2023oi" ;

    public SwerveRobot2023OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, SubsystemName, GamePadType.Swerve, db, true) ;
    }
}
