package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sds.Mk4iSwerveModuleHelper;
import frc.robot.sds.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public static SwerveModulePosition[] modulePositions;

    private final AHRS gyroscope;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    
    

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private ChassisSpeeds autoChassisSpeeds;

    private final SwerveDriveOdometry odometry;

    public static Boolean isInput;

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4iSwerveModuleHelper.createNeo(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET
        );


        gyroscope = new AHRS(I2C.Port.kMXP);

        // Uncomment one of these if an offset is needed. From drive station perspective:
        gyroscope.setAngleAdjustment(0);
        //Left side start
        //gyroscope.setAngleAdjustment(-60);
        //Right side start
        //gyroscope.setAngleAdjustment(60);

        modulePositions = new SwerveModulePosition[] {frontLeftModule.getPosition(),frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};

        odometry = new SwerveDriveOdometry(kinematics, gyroscope.getRotation2d(), modulePositions);

        // shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        // shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        // shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

        // AutoBuilder.configureHolonomic(
        // this::getPosition,
        // this::resetOdometry,
        // this::robotOrientedSpeeds,
        // this::driveRobotOriented,
        // new HolonomicPathFollowerConfig(
        //         new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
        //         new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
        //         4.5, // Max module speed, in m/s
        //         0.431,
        //         new ReplanningConfig()),
        // () -> {
        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;},
        // this);

        

    }

    public void zeroGyroscope() {
        gyroscope.reset();
        gyroscope.setAngleAdjustment(0);
        odometry.resetPosition(
                gyroscope.getRotation2d(), modulePositions,
                odometry.getPoseMeters()
        );
    }

    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(gyroscope.getRotation2d(), modulePositions, pose);
    }

    public Rotation2d getRotation() {
        return gyroscope.getRotation2d();
        //return odometry.getPoseMeters().getRotation();
    }

    public Pose2d getPosition() {
        return odometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        isInput = true;
    }

    public ChassisSpeeds robotOrientedSpeeds(){
       autoChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation());
       return autoChassisSpeeds;
    }

    public void driveRobotOriented(ChassisSpeeds autoChassisSpeeds) {
        this.chassisSpeeds = autoChassisSpeeds;
    }

    @Override
    public void periodic() {

        modulePositions = new SwerveModulePosition[] {frontLeftModule.getPosition(),frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};
        
        odometry.update(gyroscope.getRotation2d(),
                modulePositions
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }

}
