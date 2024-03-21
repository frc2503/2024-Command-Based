package frc.robot.vision;

import edu.wpi.first.wpilibj.DriverStation;

public class AprilTagIds {
    public static int getSpeakerTagId() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return 7;
        } else {
            return 4;
        }
    }
}