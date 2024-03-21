package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoDriveCommand extends Command {

    private DrivetrainSubsystem driveTrain;
    private double driveDistance;

    public AutoDriveCommand(DrivetrainSubsystem driveTrain) {
        this.driveTrain = driveTrain;
        //If starting in center
        driveDistance = -.05;
        //If starting from a side
        //driveDistance = -1;
    }

    @Override
    public void execute() {
        driveTrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveDistance,
                0,
                0,
                driveTrain.getRotation()));
    }

    @Override
    public boolean isFinished() {
        if (driveTrain.getPosition().getX() <= driveDistance) {
            System.out.println("Auto driving done");
            driveTrain.drive(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, driveTrain.getRotation()));
            return true;
        } else {
            return false;
        }
    }
    
}
