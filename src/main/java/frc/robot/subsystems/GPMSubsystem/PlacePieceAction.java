package frc.robot.subsystems.GPMSubsystem;

import java.util.Optional;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.oi.OIDevice;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.SwerveRobot2023OISubsystem;

public class PlacePieceAction extends Action {
    public enum Slot {
        Left,
        Middle,
        Right
    } ;

    public enum Height {
        Bottom,
        Middle,
        Top
    } ;

    private enum State {
        WaitingForTag,
        PositionDriveBase,
        PlaceGamePiece,
        Error
    }

    // private GPMSubsystem gpm_ ;
    private SwerveBaseSubsystem swerve_ ;
    private SwerveRobot2023OISubsystem oi_ ;
    private LimeLightSubsystem ll_ ;
    private State state_ ;
    private SwerveDriveToPoseAction goto_pose_ ;
    private int april_tag_id_ ;

    public PlacePieceAction(GPMSubsystem gpm, SwerveBaseSubsystem swerve, SwerveRobot2023OISubsystem oi, LimeLightSubsystem ll, int id, Slot slot, Height height) throws Exception {
        super(gpm.getRobot().getMessageLogger()) ;

        // gpm_ = gpm ;
        swerve_ = swerve ;
        oi_ = oi ;
        ll_ = ll ;
        april_tag_id_ = id ;

        Pose2d target = computeTargetLocation(id, slot, height) ;
        goto_pose_ = new SwerveDriveToPoseAction(swerve, target) ;
    }

    @Override
    public void start() {
        state_ = State.WaitingForTag ;
    }

    @Override
    public void run() {
        switch(state_) {
            case WaitingForTag:
                runWaitingForTag() ;
                break ;

            case PositionDriveBase:
                runPositonDriveBase() ;
                break ;

            case PlaceGamePiece:
                runPlaceGamePiece() ;
                break ;

            case Error:
                runError() ;
                break ;
        }
    }

    public String toString(int n) {
        return spaces(n) + "PlacePieceAction" ;
    }    

    private Pose2d computeTargetLocation(int id, PlacePieceAction.Slot slot, PlacePieceAction.Height height) {
        Pose2d target = null ;

        AprilTagFieldLayout layout = swerve_.getRobot().getAprilTags() ;
        Optional<Pose3d> pose = layout.getTagPose(id) ;
        if (pose.isPresent()) {
            target = pose.get().toPose2d() ;
        }

        return target ;
    }

    private void runWaitingForTag() {
        if (ll_.hasAprilTag(april_tag_id_)) {
            //
            // We have found the april tag of interest
            //
            OIDevice dev = oi_.getDevice(0) ;
            dev.disable() ;

            swerve_.setAction(goto_pose_) ;
            state_ = State.PositionDriveBase ;
        }
    }

    private void runPositonDriveBase() {   
        if (goto_pose_.isDone()) {
            state_ = State.PlaceGamePiece ;
        }
    }

    private void runPlaceGamePiece() {
        OIDevice dev = oi_.getDevice(0) ;
        dev.enable() ;
        setDone() ;
    }

    private void runError() {
        OIDevice dev = oi_.getDevice(0) ;
        dev.enable() ;
        setDone() ;
    }
}
