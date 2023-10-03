package frc.trigon.robot.subsystems.sideshooter.simulationsideshooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.sideshooter.SideShooterConstants;

public class SimulationSideShooterConstants extends SideShooterConstants {
    static final double MAX_VOLTAGE = 12;

    private static final DCMotor
            ANGLE_MOTOR_GEARBOX = DCMotor.getNeo550(1),
            SHOOTING_MOTOR_GEARBOX = DCMotor.getFalcon500(1);
    private static final double
            ANGLE_MOTOR_GEAR_RATIO = 270,
            SHOOTING_MOTOR_GEAR_RATIO = 6.66;
    private static final double SHOOTING_MOTOR_MOMENT_OF_INERTIA = 0.0032;
    private static final double SIDE_SHOOTER_MASS = 4;
    private static final Rotation2d
            MINIMUM_ARM_ANGLE = Rotation2d.fromDegrees(0),
            MAXIMUM_ARM_ANGLE = Rotation2d.fromDegrees(90);
    private static final boolean ANGLE_MOTOR_SIMULATE_GRAVITY = false;
    static final SingleJointedArmSim ANGLE_MOTOR_SIMULATION = new SingleJointedArmSim(
            ANGLE_MOTOR_GEARBOX,
            ANGLE_MOTOR_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(SideShooterConstants.SIDE_SHOOTER_LENGTH, SIDE_SHOOTER_MASS),
            SideShooterConstants.SIDE_SHOOTER_LENGTH,
            MINIMUM_ARM_ANGLE.getRadians(),
            MAXIMUM_ARM_ANGLE.getRadians(),
            ANGLE_MOTOR_SIMULATE_GRAVITY
    );
    static final FlywheelSim SHOOTING_MOTOR_SIMULATION = new FlywheelSim(
            SHOOTING_MOTOR_GEARBOX, SHOOTING_MOTOR_GEAR_RATIO, SHOOTING_MOTOR_MOMENT_OF_INERTIA
    );

    static final PIDController ANGLE_MOTOR_PID_CONTROLLER = new PIDController(
            25,
            0,
            0
    );

    private static final double
            ANGLE_MOTOR_KS = 0,
            ANGLE_MOTOR_KV = 0,
            ANGLE_MOTOR_KA = 0,
            ANGLE_MOTOR_KG = 0;
    private static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV, ANGLE_MOTOR_KA
    );

    @Override
    protected ArmFeedforward getAngleMotorFeedforward() {
        return ANGLE_MOTOR_FEEDFORWARD;
    }
}
