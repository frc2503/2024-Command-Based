package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;

public class AprilTagLock {

    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    private PIDController pidController;

    public AprilTagLock() {
        pidController = new PIDController(.01, .02, .001);
        pidController.setTolerance(.25);
        pidController.enableContinuousInput(0, 360);
        pidController.setSetpoint(0.0);
    }

    /*
     * This will get current results from the Limelight, and iterate through them to try to find the
     * angle offset from the supplied tag ID. If the tag you're looking for isn't currently visible to 
     * the Limelight this will return null.
     * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
     */
    public Double getAngleToTag(int tagId) {
        LimelightHelpers.Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;
        for (LimelightTarget_Fiducial target: results.targets_Fiducials) {
            if (target.fiducialID == tagId) {
                // "tx" is the horizontal angle offset from the provided tag
                return pidController.calculate(target.tx);
            }
        }
        return null;
    }
}
