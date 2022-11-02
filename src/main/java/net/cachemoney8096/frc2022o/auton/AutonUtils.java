package net.cachemoney8096.frc2022o.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.HashMap;
import org.tinylog.Logger;

public class AutonUtils {
  private static HashMap<String, PathPlannerTrajectory> m_cache = new HashMap<>();

  private static String cacheKey(String name, double maxVel, double maxAccel, boolean reversed) {
    return new StringBuilder()
        .append(name)
        .append(maxVel)
        .append(maxAccel)
        .append(reversed)
        .toString();
  }

  public static Pose2d initialPose(PathPlannerTrajectory trajectory) {
    return new Pose2d(
        trajectory.getInitialState().poseMeters.getTranslation(),
        trajectory.getInitialState().holonomicRotation);
  }

  public static PathPlannerTrajectory loadTrajectory(String name, double maxVel, double maxAccel) {
    return loadTrajectory(name, maxVel, maxAccel, false);
  }

  public static PathPlannerTrajectory loadTrajectory(
      String name, double maxVel, double maxAccel, boolean reversed) {
    String cacheKey_ = cacheKey(name, maxVel, maxAccel, reversed);
    if (m_cache.containsKey(cacheKey_)) {
      return m_cache.get(cacheKey_);
    }

    PathPlannerTrajectory trajectory = PathPlanner.loadPath(name, maxVel, maxAccel, reversed);

    if (trajectory == null) {
      Logger.tag("Auton Path").error("Failed to load trajectory: {}", name);
    }

    Logger.tag("Auton Path")
        .trace(
            "Loaded auton {}, max velocity: {}, max accel: {}, reversed: {}, starting state {}",
            name,
            maxVel,
            maxAccel,
            reversed,
            trajectory.getInitialState());

    /*
     * Store trajectories. This is done since multiple autons may load the same trajectories,
     * no need to load it again and again. If anything funky with trajectories is happening,
     * comment out the below. (e.g. I don't see any stored state in the trajectory, but it
     * is not clear if that is true, and if calling the trajectory stores any state.
     * Investigate later.)
     */
    m_cache.put(cacheKey_, trajectory);

    return trajectory;
  }
}
