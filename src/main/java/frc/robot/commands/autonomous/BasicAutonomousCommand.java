package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class BasicAutonomousCommand extends SequentialCommandGroup {
    public BasicAutonomousCommand(DrivetrainSubsystem driveTrain, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        addCommands(
            new AutoZeroCommand(elevator, endEffector),
            new AutoStartShootCommand(endEffector, elevator),
            new AutoShootCommand(endEffector),
            new AutoDriveCommand(driveTrain)
        );
    }
}
