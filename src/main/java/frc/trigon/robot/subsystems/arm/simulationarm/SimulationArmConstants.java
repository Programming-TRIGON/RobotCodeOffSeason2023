package frc.trigon.robot.subsystems.arm.simulationarm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.trigon.robot.subsystems.arm.ArmConstants;

public class SimulationArmConstants extends ArmConstants {
    static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    private static final DCMotor ELEVATOR_MOTORS = DCMotor.getNEO(2);
    private static final double ELEVATOR_GEAR_RATIO = 13.3;
    private static final double ELEVATOR_CARTRIDGE_MASS = 8.1;
    private static final double ELEVATOR_DRUM_RADIUS = 0.06;
    private static final boolean ELEVATOR_SIMULATE_GRAVITY = true;
    static final double ELEVATOR_METERS_PER_REVOLUTION = 0.1;
    static final ElevatorSim ELEVATOR_SIMULATION = new ElevatorSim(
            ELEVATOR_MOTORS,
            ELEVATOR_GEAR_RATIO,
            ELEVATOR_CARTRIDGE_MASS,
            ELEVATOR_DRUM_RADIUS,
            ArmConstants.RETRACTED_ARM_LENGTH,
            ArmConstants.MAX_ARM_LENGTH,
            ELEVATOR_SIMULATE_GRAVITY
    );

    private static final DCMotor ANGLE_MOTORS = DCMotor.getNEO(3);
    private static final double ANGLE_GEAR_RATIO = 97.4;
    private static final double ARM_MASS = 8.1;
    private static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(-45),
            MAX_ANGLE = Rotation2d.fromDegrees(180);
    private static final boolean ANGLE_SIMULATE_GRAVITY = true;
    static final SingleJointedArmSim ANGLE_SIMULATION = new SingleJointedArmSim(
            ANGLE_MOTORS,
            ANGLE_GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ArmConstants.RETRACTED_ARM_LENGTH, ARM_MASS),
            ArmConstants.RETRACTED_ARM_LENGTH,
            MIN_ANGLE.getRadians(),
            MAX_ANGLE.getRadians(),
            ANGLE_SIMULATE_GRAVITY
    );

    private static final double
            ANGLE_MOTOR_KS = 0.010169 / 5,
            ANGLE_MOTOR_KV = 0.053077 / 5,
            ANGLE_MOTOR_KG = 0.071665 / 5,
            ELEVATOR_MOTOR_KS = 10.169 / 7.5,
            ELEVATOR_MOTOR_KV = 5.3077 / 7.5,
            ELEVATOR_MOTOR_KG = 7.1665 / 7.5;
    private static final double
            ANGLE_MOTOR_KP = 1,
            ANGLE_MOTOR_KI = 0,
            ANGLE_MOTOR_KD = 0,
            ELEVATOR_MOTOR_KP = 18,
            ELEVATOR_MOTOR_KI = 0,
            ELEVATOR_MOTOR_KD = 0;
    private static final ArmFeedforward ANGLE_MOTOR_FEEDFORWARD = new ArmFeedforward(
            ANGLE_MOTOR_KS, ANGLE_MOTOR_KG, ANGLE_MOTOR_KV
    );
    private static final ElevatorFeedforward ELEVATOR_MOTOR_FEEDFORWARD = new ElevatorFeedforward(
            ELEVATOR_MOTOR_KS, ELEVATOR_MOTOR_KG, ELEVATOR_MOTOR_KV
    );
    static final PIDController
            ANGLE_PID_CONTROLLER = new PIDController(ANGLE_MOTOR_KP, ANGLE_MOTOR_KI, ANGLE_MOTOR_KD),
            ELEVATOR_PID_CONTROLLER = new PIDController(ELEVATOR_MOTOR_KP, ELEVATOR_MOTOR_KI, ELEVATOR_MOTOR_KD);

    @Override
    protected ArmFeedforward getAngleMotorFeedforward() {
        return ANGLE_MOTOR_FEEDFORWARD;
    }

    @Override
    protected ElevatorFeedforward getElevatorMotorFeedforward() {
        return ELEVATOR_MOTOR_FEEDFORWARD;
    }
}
