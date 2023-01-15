package frc.robot.subsystems.GPMSubsystem;

import java.util.Optional;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.oi.OIDevice;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
        Error
    }

    private GPMSubsystem gpm_ ;
    private SwerveBaseSubsystem swerve_ ;
    private SwerveRobot2023OISubsystem oi_ ;
    private LimeLightSubsystem ll_ ;
    private int id_ ;
    private Slot slot_ ;
    private Height height_ ;
    private State state_ ;
    private HolonomicDriveController ctrl_ ;
    private Pose2d target_position_ ;

    public PlacePieceAction(GPMSubsystem gpm, SwerveBaseSubsystem swerve, SwerveRobot2023OISubsystem oi, LimeLightSubsystem ll, int id, Slot slot, Height height) throws Exception {
        super(gpm.getRobot().getMessageLogger()) ;

        gpm_ = gpm ;
        swerve_ = swerve ;
        oi_ = oi ;
        ll_ = ll ;
        id_ = id ;
        slot_ = slot ;
        height_ = height ;

        try {
            ctrl_ = createDriveController() ;
        }
        catch(Exception ex) {
            MessageLogger logger = swerve_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("error craating HolonomicDriveController for PlacePieceAction ").endMessage();
            logger.logStackTrace(ex.getStackTrace()) ;
            throw ex ;
        }
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

            case Error:
                runError() ;
                break ;
        }
    }

    public String toString(int n) {
        return spaces(n) + "PlacePieceAction" ;
    }    

    private HolonomicDriveController createDriveController() throws BadParameterTypeException, MissingParameterException {
        HolonomicDriveController ctrl = null ;
        double kp, ki, kd ;

        double maxv = swerve_.getSettingsValue("physical:max-angular-speed").getDouble() ;
        double maxa = swerve_.getSettingsValue("physical:max-angular-accel").getDouble() ;

        kp = swerve_.getSettingsValue("pid:xctrl:kp").getDouble() ;
        ki = swerve_.getSettingsValue("pid:xctrl:ki").getDouble() ;
        kd = swerve_.getSettingsValue("pid:xctrl:kd").getDouble() ;
        PIDController xctrl = new PIDController(kp, ki, kd) ;

        kp = swerve_.getSettingsValue("pid:yctrl:kp").getDouble() ;
        ki = swerve_.getSettingsValue("pid:yctrl:ki").getDouble() ;
        kd = swerve_.getSettingsValue("pid:yctrl:kd").getDouble() ;
        PIDController yctrl = new PIDController(kp, ki, kd) ;

        kp = swerve_.getSettingsValue("pid:rotation:kp").getDouble() ;
        ki = swerve_.getSettingsValue("pid:rotation:ki").getDouble() ;
        kd = swerve_.getSettingsValue("pid:rotation:kd").getDouble() ;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxv, maxa) ;
        ProfiledPIDController thetactrl = new ProfiledPIDController(kp, ki, kd, constraints) ;
        thetactrl.enableContinuousInput(-Math.PI, Math.PI);
        
        ctrl = new HolonomicDriveController(xctrl, yctrl, thetactrl) ;
        ctrl.setEnabled(true);

        return ctrl ;
    }

    private double slotOffset() {
        //
        // TODO: compute the offset (left or right) from the april tag to the
        //       placement position based on the id, slot, and height
        return -12.0 ;
    }

    private Pose2d computeTargetLocation() {
        Pose2d target = null ;

        AprilTagFieldLayout layout = swerve_.getRobot().getAprilTags() ;
        Optional<Pose3d> pose = layout.getTagPose(id_) ;
        if (pose.isPresent()) {
            target = pose.get().toPose2d() ;

            Translation2d t2d = new Translation2d(0.0, slotOffset()) ;
            target = target.plus(new Transform2d(t2d, Rotation2d.fromDegrees(0.0))) ;
        }

        return target ;
    }

    private void runWaitingForTag() {
        if (ll_.hasAprilTag(id_)) {
            //
            // We have found the april tag of interest
            //
            OIDevice dev = oi_.getDevice(0) ;
            dev.disable() ;

            target_position_ = computeTargetLocation() ;
            if (target_position_ == null) {
                MessageLogger logger = swerve_.getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error).add("error computing robot position for placement") ;
                logger.add("id", id_) ;
                logger.add("slot", slot_.toString()) ;
                logger.add("height", height_.toString()) ;
                logger.endMessage();
                state_ = State.Error ;
            }
            state_ = State.PositionDriveBase ;
        }
    }

    private void runPositonDriveBase() {   
        //
        // TODO: draw a straight line, trapezoidal profile path to the target position
        //
        ChassisSpeeds speed = ctrl_.calculate(swerve_.getPose(), target_position_, 1.0, target_position_.getRotation()) ;
        swerve_.drive(speed) ;
    }

    private void runError() {
        OIDevice dev = oi_.getDevice(0) ;
        dev.enable() ;
        setDone() ;
    }
}
