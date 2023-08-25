package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.robotposesources.PoseSourceConstants;

import java.util.Dictionary;

public class FieldConstants {
    public static final double
            FIELD_LENGTH_METERS = 16.54175,
            FIELD_WIDTH_METERS = 8.02;
    public static final double
            INNER_SUBSTATION_Y = 6.13010,
            OUTER_SUBSTATION_Y = 7.44483;
    public static final double ROBOT_LENGTH_WITH_BUMPERS = ((6.98 * 2 )+ 65) / 100;
    private static final double IN_FRONT_OF_GRID_X = 1.37752 + (ROBOT_LENGTH_WITH_BUMPERS / 2);
    private static final double COLUMN_DISTANCE_FROM_MIDDLE_COLUMN = 0.55983;
    private static final double
            LEFT_GRID_MIDDLE_COLUMN_Y = PoseSourceConstants.TAGS_ID_TO_POSE.get(6).getY(),
            MIDDLE_GRID_MIDDLE_COLUMN_Y = PoseSourceConstants.TAGS_ID_TO_POSE.get(7).getY(),
            RIGHT_GRID_MIDDLE_COLUMN_Y = PoseSourceConstants.TAGS_ID_TO_POSE.get(8).getY();
    private static final Rotation2d IN_FRONT_OF_GRID_ROTATION = Rotation2d.fromRotations(0.5);

    public enum GridAlignment {
        LEFT_GRID_LEFT_COLUMN(IN_FRONT_OF_GRID_X, LEFT_GRID_MIDDLE_COLUMN_Y + COLUMN_DISTANCE_FROM_MIDDLE_COLUMN, IN_FRONT_OF_GRID_ROTATION),
        LEFT_GRID_MIDDLE_COLUMN(IN_FRONT_OF_GRID_X, LEFT_GRID_MIDDLE_COLUMN_Y, IN_FRONT_OF_GRID_ROTATION),
        LEFT_GRID_RIGHT_COLUMN(IN_FRONT_OF_GRID_X, LEFT_GRID_MIDDLE_COLUMN_Y - COLUMN_DISTANCE_FROM_MIDDLE_COLUMN, IN_FRONT_OF_GRID_ROTATION),
        MIDDLE_GRID_LEFT_COLUMN(IN_FRONT_OF_GRID_X, MIDDLE_GRID_MIDDLE_COLUMN_Y + COLUMN_DISTANCE_FROM_MIDDLE_COLUMN, IN_FRONT_OF_GRID_ROTATION),
        MIDDLE_GRID_MIDDLE_COLUMN(IN_FRONT_OF_GRID_X, MIDDLE_GRID_MIDDLE_COLUMN_Y, IN_FRONT_OF_GRID_ROTATION),
        MIDDLE_GRID_RIGHT_COLUMN(IN_FRONT_OF_GRID_X, MIDDLE_GRID_MIDDLE_COLUMN_Y - COLUMN_DISTANCE_FROM_MIDDLE_COLUMN, IN_FRONT_OF_GRID_ROTATION),
        RIGHT_GRID_LEFT_COLUMN(IN_FRONT_OF_GRID_X, RIGHT_GRID_MIDDLE_COLUMN_Y + COLUMN_DISTANCE_FROM_MIDDLE_COLUMN, IN_FRONT_OF_GRID_ROTATION),
        RIGHT_GRID_MIDDLE_COLUMN(IN_FRONT_OF_GRID_X, RIGHT_GRID_MIDDLE_COLUMN_Y, IN_FRONT_OF_GRID_ROTATION),
        RIGHT_GRID_RIGHT_COLUMN(IN_FRONT_OF_GRID_X, RIGHT_GRID_MIDDLE_COLUMN_Y - COLUMN_DISTANCE_FROM_MIDDLE_COLUMN, IN_FRONT_OF_GRID_ROTATION);

        public final Pose2d inFrontOfGridPose;
        public final int gridNumber;
        public final int columnNumber;

        GridAlignment(double x, double y, Rotation2d angle) {
            this(new Pose2d(x, y, angle));
        }

        GridAlignment(Pose2d inFrontOfGridPose) {
            this.inFrontOfGridPose = inFrontOfGridPose;
            final int index = ordinal();

            gridNumber = index / 3 + 1;
            columnNumber = index % 3 + 1;
        }

        public static GridAlignment getGridAlignment(int gridNumber, int columnNumber) {
            return values()[(gridNumber - 1) * 3 + columnNumber - 1];
        }
    }
}
