package frc.trigon.robot.constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.commands.Commands;
import frc.trigon.robot.subsystems.roller.Roller;
import frc.trigon.robot.subsystems.sideshooter.SideShooter;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;
import frc.trigon.robot.utilities.FilesHandler;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class AutonomousConstants {
    public static final HashMap<String, Command> EVENT_MAP = new HashMap<>();
    public static final List<String> AUTONOMOUS_PATHS_NAMES = new ArrayList<>();
    public static final HashMap<String, List<PathPlannerTrajectory>> PRELOADED_PATHS = new HashMap<>();
    public static final PathConstraints
            AUTONOMOUS_PATH_CONSTRAINTS = new PathConstraints(3, 2),
            DRIVE_TO_GRID_ALIGNMENT_CONSTRAINTS = new PathConstraints(3.5, 6);

    private static final File PATH_PLANNER_DIRECTORY = new File(FilesHandler.DEPLOY_PATH + "pathplanner");

    static {
        configureEventMap();
        configureAutonomousPathsNames();
    }

    private static void configureEventMap() {
        EVENT_MAP.put("place-cone-3", Commands.getPlaceConeAtHighForAutoCommand());
        EVENT_MAP.put("cube-collection", Commands.getNonAssistedCubeCollectionCommand());
        EVENT_MAP.put("close-collection", Commands.getFullCollectionCloseCommand());
        EVENT_MAP.put("shoot-cube-3", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.HIGH, true));
        EVENT_MAP.put("shoot-cube-2", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.MIDDLE, true));
        EVENT_MAP.put("cube-3-afar", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.HIGH_FROM_AFAR, true));
        EVENT_MAP.put("cube-2-afar", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.MIDDLE_FROM_AFAR, true));
        EVENT_MAP.put("cube-1-afar", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.HYBRID_FROM_AFAR, true));
        EVENT_MAP.put("cube-over-ramp", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.SHOOT_OVER_RAMP, true));
        EVENT_MAP.put("stop-shooting", SideShooter.getInstance().getSetTargetShooterPowerCommand(0));
        EVENT_MAP.put("close-roller", Roller.getInstance().getFullCloseCommand());
    }

    private static void configureAutonomousPathsNames() {
        if (!PATH_PLANNER_DIRECTORY.exists())
            return;

        final File[] files = PATH_PLANNER_DIRECTORY.listFiles();
        if (files == null)
            return;

        for (File file : files) {
            if (!file.isFile() || !file.getName().endsWith(".path"))
                continue;

            AUTONOMOUS_PATHS_NAMES.add(file.getName().substring(0, file.getName().length() - 5));
        }
    }
}
