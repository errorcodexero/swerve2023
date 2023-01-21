package org.xero1425.base.subsystems.swerve.common;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDriveToPoseAction extends SwerveDriveAction {
    private Pose2d target_ ;
    
    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d target) {
        super(subsys) ;
    }

    @Override
    public void start() {

    }

    @Override
    public void run() {
        
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveToPoseAction " + target_.getTranslation().toString() + ", " + 
        target_.getRotation().getDegrees() + " deg" ;
    }
}
