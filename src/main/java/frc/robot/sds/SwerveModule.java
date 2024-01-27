package frc.robot.sds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    double getDriveVelocity();

    double getDriveDistance();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    default SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getSteerAngle()));
    }

}