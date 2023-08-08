package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotConstants {
    public static final RobotType ROBOT_TYPE = RobotType.STATIC;
    public static final boolean IS_REPLAY = false;

    public enum RobotType {
        STATIC(Filesystem.getDeployDirectory().getPath() + "/logs/"),
        TESTING(Filesystem.getDeployDirectory().getPath() + "/logs/"),
        SIMULATION(Filesystem.getDeployDirectory().getPath() + "/logs/");

        public final String loggingPath;

        RobotType(String loggingPath) {
            this.loggingPath = loggingPath;
        }
    }
}
