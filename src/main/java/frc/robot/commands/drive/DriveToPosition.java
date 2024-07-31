package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MechanismControl.MaplePIDController;
import frc.robot.utils.MechanismControl.MapleProfiledPIDController;

import java.util.function.Supplier;

public class DriveToPosition extends Command {
    private final Supplier<Pose2d> desiredPoseSupplier;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final HolonomicDriveController positionController;

    public DriveToPosition(Supplier<Pose2d> desiredPoseSupplier, HolonomicDriveSubsystem driveSubsystem) {
        this.desiredPoseSupplier = desiredPoseSupplier;
        this.driveSubsystem = driveSubsystem;

        this.positionController = new HolonomicDriveController(
                new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
                new MaplePIDController(Constants.SwerveDriveChassisConfigs.chassisTranslationPIDConfig),
                new MapleProfiledPIDController(Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig, Constants.SwerveDriveChassisConfigs.chassisRotationalConstraints)
        );

        super.addRequirements(driveSubsystem);
    }


    @Override
    public void initialize() {
        getFeedBackSpeeds();
    }

    @Override
    public void execute() {
        final ChassisSpeeds feedBackSpeeds = getFeedBackSpeeds();
        driveSubsystem.runRobotCentricChassisSpeeds(feedBackSpeeds);
    }

    /**
     * @return the feed-back speed, robot-relative
     * */
    private ChassisSpeeds getFeedBackSpeeds() {
        return positionController.calculate(driveSubsystem.getPose(), desiredPoseSupplier.get(), 0, desiredPoseSupplier.get().getRotation());
    }

    @Override
    public boolean isFinished() {
        return this.positionController.atReference();
    }
}
