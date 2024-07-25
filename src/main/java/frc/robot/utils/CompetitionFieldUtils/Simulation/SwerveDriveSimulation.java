package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.IO.GyroIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.OdometryThread;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleMaths.SwerveStateProjection;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.function.Consumer;

import static frc.robot.Constants.RobotPhysicsSimulationConfigs.*;
import static frc.robot.Constants.ChassisDefaultConfigs.*;

/**
 * simulates the behavior of our robot
 * it has all the physics behavior as a simulated holonomic chassis
 * in addition to that, it simulates the swerve module behaviors
 * the class is like the bridge between ModuleIOSim and HolonomicChassisSimulation
 * it reads the motor power from ModuleIOSim
 * and feed the result of the physics simulation back to ModuleIOSim, to simulate the odometry encoders' readings
 * TODO write this class
 * */
public class SwerveDriveSimulation extends HolonomicChassisSimulation {
    private final GyroIOSim gyroIOSim;
    private final ModuleIOSim[] modules;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<Pose2d> resetOdometryCallBack;
    public SwerveDriveSimulation(
            MapleConfigFile.ConfigBlock chassisGeneralInfoBlock,
            GyroIOSim gyroIOSim,
            ModuleIOSim frontLeft, ModuleIOSim frontRight, ModuleIOSim backLeft, ModuleIOSim backRight,
            SwerveDriveKinematics kinematics,
            Pose2d startingPose,
            Consumer<Pose2d> resetOdometryCallBack
    ) {
        super(new RobotProfile(chassisGeneralInfoBlock), startingPose);
        this.gyroIOSim = gyroIOSim;
        this.modules = new ModuleIOSim[] {frontLeft, frontRight, backLeft, backRight};
        this.kinematics = kinematics;
        this.resetOdometryCallBack = resetOdometryCallBack;
        resetOdometryToActualRobotPose();
    }

    public void resetOdometryToActualRobotPose() {
        resetOdometryCallBack.accept(getObjectOnFieldPose2d());
    }

    @Override
    public void updateSimulationSubPeriod(int iterationNum, double subPeriodSeconds) {
        // TODO: improve the simulation method
        for (ModuleIOSim module:modules)
            module.updateSim(subPeriodSeconds);
        final ChassisSpeeds swerveWheelFreeSpeeds = kinematics.toChassisSpeeds(
                Arrays.stream(modules)
                        .map(moduleIOSim -> moduleIOSim.getFreeSwerveSpeed(profile.robotMaxVelocity))
                        .toArray(SwerveModuleState[]::new)
        );
        super.simulateChassisBehaviorWithRobotRelativeSpeeds(swerveWheelFreeSpeeds);

        final ChassisSpeeds instantVelocityRobotRelative = getMeasuredChassisSpeedsRobotRelative();
        final SwerveModuleState[] actualModuleFloorSpeeds = kinematics.toSwerveModuleStates(instantVelocityRobotRelative);

        updateGyroSimulationResults(
                gyroIOSim,
                super.getObjectOnFieldPose2d().getRotation(),
                super.getAngularVelocity(),
                iterationNum
        );
        for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++)
            updateModuleSimulationResults(
                    modules[moduleIndex],
                    actualModuleFloorSpeeds[moduleIndex],
                    profile.robotMaxVelocity,
                    iterationNum, subPeriodSeconds
            );
    }

    private static void updateGyroSimulationResults(
            GyroIOSim gyroIOSim,
            Rotation2d currentFacing,
            double angularVelocityRadPerSec,
            int iterationNum) {
        final GyroIOSim.GyroPhysicsSimulationResults results = gyroIOSim.gyroPhysicsSimulationResults;
        results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec;
        results.odometryYawPositions[iterationNum] = currentFacing;
        results.hasReading = true;
    }

    private static void updateModuleSimulationResults(
            ModuleIOSim module,
            SwerveModuleState actualModuleFloorSpeed,
            double robotMaxVelocity,
            int simulationIteration, double periodSeconds) {
        final SwerveModuleState moduleFreeSwerveSpeed = module.getFreeSwerveSpeed(robotMaxVelocity);
        final ModuleIOSim.SwerveModulePhysicsSimulationResults results = module.physicsSimulationResults;
        final double projectedModuleFloorSpeedMetersPerSecond = SwerveStateProjection.project(
                actualModuleFloorSpeed,
                moduleFreeSwerveSpeed.angle
        );

        results.driveWheelFinalVelocityRevolutionsPerSec = getActualDriveMotorRotterSpeedRevPerSec(
                projectedModuleFloorSpeedMetersPerSecond,
                moduleFreeSwerveSpeed.speedMetersPerSecond
        );
        results.odometrySteerPositions[simulationIteration] = moduleFreeSwerveSpeed.angle;
        results.driveWheelFinalRevolutions += results.driveWheelFinalVelocityRevolutionsPerSec * periodSeconds;
        results.odometryDriveWheelRevolutions[simulationIteration] = results.driveWheelFinalRevolutions;
    }


    private static double getActualDriveMotorRotterSpeedRevPerSec(double moduleSpeedProjectedOnSwerveHeadingMPS, double moduleFreeSpeedMPS) {
        // TODO: move configs to Constants
        final double FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED = 0.8, rotterSpeedMPS;
        if (Math.abs(moduleFreeSpeedMPS - moduleSpeedProjectedOnSwerveHeadingMPS) / Math.abs(moduleFreeSpeedMPS) < 0.5)
            rotterSpeedMPS = moduleSpeedProjectedOnSwerveHeadingMPS;
        else rotterSpeedMPS =
                moduleSpeedProjectedOnSwerveHeadingMPS * FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED
                        + moduleFreeSpeedMPS * (1 - FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED);

        final double rotterSpeedRadPerSec = rotterSpeedMPS / DEFAULT_WHEEL_RADIUS_METERS;
        return Units.radiansToRotations(rotterSpeedRadPerSec);
    }

    public static final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.measurementTimeStamps = new double[SIM_ITERATIONS_PER_ROBOT_PERIOD];
            final double robotStartingTimeStamps = Logger.getTimestamp(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs/SIM_ITERATIONS_PER_ROBOT_PERIOD;
            for (int i =0; i < SIM_ITERATIONS_PER_ROBOT_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }
}
