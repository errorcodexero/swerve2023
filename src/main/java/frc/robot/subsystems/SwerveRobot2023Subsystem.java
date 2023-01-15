package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

public class SwerveRobot2023Subsystem extends RobotSubsystem {
    
    private SwerveBaseSubsystem db_;
    private SwerveRobot2023OISubsystem oi_ ;
    private LimeLightSubsystem limelight_ ;

    public SwerveRobot2023Subsystem(XeroRobot robot) throws Exception {
        super(robot, "SwerveRobot2023Subsystem") ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve") ;
        addChild(db_) ;

        oi_ = new SwerveRobot2023OISubsystem(this, db_) ;
        addChild(oi_) ;

        limelight_ = new LimeLightSubsystem(this, "limelight") ;
        addChild(limelight_) ;
        db_.setVision(limelight_);
    }

    @Override
    public void computeMyState() {
    }
}