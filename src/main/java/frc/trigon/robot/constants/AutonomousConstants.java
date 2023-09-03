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
            AUTONOMOUS_PATH_CONSTRAINTS = new PathConstraints(3, 3),
            DRIVE_TO_GRID_ALIGNMENT_CONSTRAINTS = new PathConstraints(3.5, 6);

    private static final File PATH_PLANNER_DIRECTORY = new File(FilesHandler.DEPLOY_PATH + "pathplanner");

    static {
        configureEventMap();
        configureAutonomousPathsNames();
    }

    private static void configureEventMap() {
        EVENT_MAP.put("place-cone-3", Commands.getPlaceConeAtHighForAutoCommand());
        EVENT_MAP.put("cude-collection", Commands.getNonAssistedCubeCollectionCommand());
        EVENT_MAP.put("close-collection", Commands.getFullCollectionCloseCommand());
        EVENT_MAP.put("close-roller", Roller.getInstance().getCloseCommand());
        EVENT_MAP.put("shoot-cude-3", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.HIGH));
        EVENT_MAP.put("prepare-shoot-cude-3", SideShooter.getInstance().getSetTargetShooterAngleCommand(SideShooterConstants.SideShooterState.HIGH.angle));
        EVENT_MAP.put("prepare-shoot-cude-1-from-afar", SideShooter.getInstance().getSetTargetShooterAngleCommand(SideShooterConstants.SideShooterState.LOW_FROM_AFAR.angle));
        EVENT_MAP.put("shoot-cude-1-from-afar", SideShooter.getInstance().getSetTargetShooterStateCommand(SideShooterConstants.SideShooterState.LOW_FROM_AFAR));
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
