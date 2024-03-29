package frc.robot.sds;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface DriveController {
    MotorController getDriveMotor();
    void setReferenceVoltage(double voltage);
    double getStateVelocity();
    double getStateDistance();
}
