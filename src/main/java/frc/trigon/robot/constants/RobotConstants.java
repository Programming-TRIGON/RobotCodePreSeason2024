package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotConstants {
    public static final RobotType ROBOT_TYPE = RobotType.SIMULATION;
    public static final boolean IS_REPLAY = false;
    public static final double PERIODIC_TIME_SECONDS = 0.02;
    private static final String DEPLOY_PATH = Filesystem.getDeployDirectory().getPath() + "/";

    public enum RobotType {
        KABLAMA("/media/sda1/logs/"),
        TRIHARD(DEPLOY_PATH + "logs/"),
        SIMULATION(DEPLOY_PATH + "logs/");

        public final String loggingPath;

        RobotType(String loggingPath) {
            this.loggingPath = loggingPath;
        }
    }
}
