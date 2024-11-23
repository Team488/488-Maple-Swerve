package frc.robot.subsystems.vision.apriltags;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects.NoteOnFieldSimulated;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class ApriltagVisionIOSim extends AprilTagVisionIOReal {
    private final VisionSystemSim visionSystemSim;
    private final PhotonCameraSim[] camerasSim;
    private final Supplier<Pose2d> robotActualPoseInSimulationSupplier;

    public ApriltagVisionIOSim(
            List<PhotonCameraProperties> cameraProperties,
            AprilTagFieldLayout aprilTagFieldLayout,
            Supplier<Pose2d> robotActualPoseInSimulationSupplier) {
        super(cameraProperties);

        this.robotActualPoseInSimulationSupplier = robotActualPoseInSimulationSupplier;
        this.visionSystemSim = new VisionSystemSim("main");
        visionSystemSim.addAprilTags(aprilTagFieldLayout);
        this.addNotesVisionTargets();

        camerasSim = new PhotonCameraSim[cameraProperties.size()];

        for (int i = 0; i < cameraProperties.size(); i++) {
            final PhotonCameraSim cameraSim = new PhotonCameraSim(
                    super.cameras[i], cameraProperties.get(i).getSimulationProperties());
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            cameraSim.enableDrawWireframe(true);
            visionSystemSim.addCamera(camerasSim[i] = cameraSim, cameraProperties.get(i).robotToCamera);
        }
    }

    private static final Translation2d[] NOTE_INITIAL_POSITIONS = new Translation2d[] {
        new Translation2d(2.9, 4.1),
        new Translation2d(2.9, 5.55),
        new Translation2d(2.9, 7),
        new Translation2d(8.27, 0.75),
        new Translation2d(8.27, 2.43),
        new Translation2d(8.27, 4.1),
        new Translation2d(8.27, 5.78),
        new Translation2d(8.27, 7.46),
        new Translation2d(13.64, 4.1),
        new Translation2d(13.64, 5.55),
        new Translation2d(13.64, 7),
    };

    private void addNotesVisionTargets() {
        for (Translation2d notePosition : NOTE_INITIAL_POSITIONS) {
            NoteOnFieldSimulated note = new Crescendo2024FieldObjects.NoteOnFieldSimulated(notePosition);
            TargetModel noteModel = new TargetModel(note.getNoteDiameter(), note.getNoteDiameter(), note.getGamePieceHeight());
            VisionTargetSim noteTarget = new VisionTargetSim(note.getPose3d(), noteModel);
            visionSystemSim.addVisionTargets(noteTarget);
        }
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        visionSystemSim.update(robotActualPoseInSimulationSupplier.get());
        super.updateInputs(inputs);
    }
}
