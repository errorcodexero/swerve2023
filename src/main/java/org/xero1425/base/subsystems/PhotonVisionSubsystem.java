package org.xero1425.base.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonVisionSubsystem extends Subsystem {
    SwerveBaseSubsystem db_ ;
    RobotPoseEstimator estimator_ ;
    AprilTagFieldLayout layout_ ;
    ArrayList<Pair<PhotonCamera, Transform3d>> cameras_ ;

    public PhotonVisionSubsystem(Subsystem parent, SwerveBaseSubsystem db) throws BadParameterTypeException, MissingParameterException {
        super(parent, "photon") ;
        double x, y, z ;
        Transform3d xform ;
        PhotonCamera cam ;

        db_ = db ;

        cameras_ = new ArrayList<Pair<PhotonCamera, Transform3d>>() ;

        cam = new PhotonCamera("cam1") ;
        x = getSettingsValue("cam1:offset:x").getDouble() ;
        y = getSettingsValue("cam1:offset:y").getDouble() ;
        z = getSettingsValue("cam1:offset:z").getDouble() ;
        xform = new Transform3d(new Translation3d(x, y, z), new Rotation3d(0.0, 0.0, 0.0)) ;
        cameras_.add(new Pair<PhotonCamera, Transform3d>(cam, xform)) ;

        estimator_ = new RobotPoseEstimator(layout_, PoseStrategy.LOWEST_AMBIGUITY, cameras_) ;
    }
    
    @Override
    public void computeState() {
        estimator_.setReferencePose(db_.getPose()) ;
        Optional<Pair<Pose3d, Double>> result = estimator_.update() ;
        if (result.isPresent()) {
            Pose2d pose = result.get().getFirst().toPose2d() ;
            db_.addVisionMeasurement(pose, getRobot().getTime()) ;
        }
    }
}
