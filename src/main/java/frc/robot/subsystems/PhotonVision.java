
package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.CameraConstants.camera1;
import frc.robot.SwerveModule;

public class PhotonVision extends SubsystemBase{
    PhotonCamera camera;
    PhotonPipelineResult lastestDetection;
    PhotonTrackedTarget target;

    Transform3d targetLocation; 
    
    boolean bestTarget;
    int targetID;
    double targetYaw = 0.0;
    double targetRange = 0.0;

    public PhotonVision(String cameraName, boolean bestTarget, int targetID) {
        this.camera = new PhotonCamera(cameraName);
        this.bestTarget = bestTarget;
        this.targetID = targetID;
    }

    @Override 
    public void periodic(){
        lastestDetection = camera.getLatestResult();
        // check for detected target to ensure null object is not accessed 
        if(lastestDetection != null) {
            // look for target depending on determination of which one you want
            if(bestTarget) { 
                target = lastestDetection.getBestTarget();
            } else {
                target = lastestDetection.getTargets().get(0);
            }
            
            // check for correct detection of target 
            if(target.getFiducialId() == targetID) {
                whenTargetSeen(target);
            } else {
                target = null;
            }
        } else {
            // destroy values to ensure false values aren't used 
            whenTargetNotSeen();
        }
    }

    // calculate information about target 
    public void whenTargetSeen(PhotonTrackedTarget target) {
        targetYaw = target.getYaw();
        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
            camera1.cameraHightOffGround, 
            camera1.targetHighoffGround, 
            Units.degreesToRadians(camera1.cameraPitch),
            Units.degreesToRadians(target.getPitch()));
    }

    // return information calculated 
    public double[] getTargetInformation() {
        double[] info = {targetYaw, targetRange};
        return info;
    }

    public void whenTargetNotSeen() {
        targetYaw = -1000;
        targetRange = -1000; 
    }

    //TODO abstract smart dash board stuff 
    public void dataToSmartdashboard() {

    }
}
