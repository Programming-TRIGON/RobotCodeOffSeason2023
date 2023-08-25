package frc.trigon.robot.subsystems.roller.simulationroller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.roller.RollerConstants;

public class SimulationRollerConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getNeo550(1),
            COLLECTION_MOTOR_GEARBOX = DCMotor.getNeo550(1);
    private static final double
            ANGLE_MOTOR_GEAR_RATIO = 81,
            COLLECTION_MOTOR_GEAR_RATIO = 13.5;
    private static final double COLLECTION_MOTOR_MOMENT_OF_INERTIA = 0.0032;
    private static final double ROLLER_MASS = 4;
    private static final Rotation2d
            MINIMUM_ARM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ARM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean ANGLE_MOTOR_SIMULATE_GRAVITY = true;
    static final SingleJointedArmSim ANGLE_MOTOR_SIMULATION = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(RollerConstants.ROLLER_LENGTH, ROLLER_MASS),
            RollerConstants.ROLLER_LENGTH,
            MINIMUM_ARM_ANGLE.getRadians(),
            MAXIMUM_ARM_ANGLE.getRadians(),
            ANGLE_MOTOR_SIMULATE_GRAVITY
    );
    static final FlywheelSim COLLECTION_MOTOR_SIMULATION = new FlywheelSim(
            COLLECTION_MOTOR_GEARBOX, COLLECTION_MOTOR_GEAR_RATIO, COLLECTION_MOTOR_MOMENT_OF_INERTIA
    );
}
