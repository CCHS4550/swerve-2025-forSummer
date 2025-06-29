package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
public interface VisionIO{

    public class VisionIOInputs{
        public Pose2d[] poseEstimates;
        public double[] timestampArray;

        public boolean hasTarget;
        public boolean hasEstimate;

        public int focusedID;

    }

    default void updateInputs(VisionIOInputs inputs) {}

}