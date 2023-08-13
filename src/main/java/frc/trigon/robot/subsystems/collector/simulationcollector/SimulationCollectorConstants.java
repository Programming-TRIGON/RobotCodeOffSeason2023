package frc.trigon.robot.subsystems.collector.simulationcollector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimulationCollectorConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final DCMotor MOTOR_GEARBOX = DCMotor.getBag(1);
    private static final double GEAR_RATIO = 49;
    private static final double MOMENT_OF_INERTIA = 0.0032;

    static final FlywheelSim MOTOR_SIMULATION = new FlywheelSim(
            MOTOR_GEARBOX, GEAR_RATIO, MOMENT_OF_INERTIA
    );
}
