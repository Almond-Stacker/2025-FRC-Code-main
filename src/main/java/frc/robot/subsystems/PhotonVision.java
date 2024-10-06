
package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.CameraConstants;

public class PhotonVision extends SubsystemBase{
    private PhotonCamera camera;
    private PhotonPipelineResult currentFrame; 
    private List<PhotonTrackedTarget> aprilTagsFound;
    
    public PhotonVision(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void periodic() {
        currentFrame = camera.getLatestResult();
        if(currentFrame.hasTargets())
        {
            aprilTagsFound = currentFrame.getTargets();
        }
        else
        {
            aprilTagsFound = null;
        }
    }

    public double calculateAzumith() {
        aprilTagsFound.get(0);
        return 0;
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}
