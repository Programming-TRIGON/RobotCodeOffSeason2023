package frc.trigon.robot.subsystems.swerve;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.kablamaswerve.KablamaSwerveConstants;
import frc.trigon.robot.subsystems.swerve.kablamaswerve.KablamaSwerveIO;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveConstants;
import frc.trigon.robot.subsystems.swerve.simulationswerve.SimulationSwerveIO;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveConstants;
import frc.trigon.robot.subsystems.swerve.trihardswerve.TrihardSwerveIO;
import frc.trigon.robot.utilities.AllianceUtilities;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    private final static Swerve INSTANCE = new Swerve();

    private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
    private final SwerveIO swerveIO;
    private final SwerveModuleIO[] modulesIO;
    private final SwerveConstants constants;
    Pose2d profiledTargetPose = null;

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve() {
        swerveIO = generateIO();
        constants = generateConstants();
        modulesIO = getModulesIO();
    }

    @Override
    public void periodic() {
        swerveIO.updateInputs(swerveInputs);
        Logger.getInstance().processInputs("Swerve", swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();

        updateNetworkTables();
    }

    /**
     * @return the swerve constants
     */
    public SwerveConstants getConstants() {
        return constants;
    }

    /**
     * @return the acceleration of the gyro in the z-axis
     */
    public double getGyroZAcceleration() {
        return swerveInputs.accelerationZ;
    }

    /**
     * @return the acceleration of the gyro in the y-axis
     */
    public double getGyroYAcceleration() {
        return swerveInputs.accelerationY;
    }

    /**
     * @return the acceleration of the gyro in the x-axis
     */
    public double getGyroXAcceleration() {
        return swerveInputs.accelerationX;
    }

    /**
     * @return the heading of the robot
     */
    public Rotation2d getHeading() {
        final double heading = swerveInputs.gyroAngleDegrees;
        final double inputtedHeading = MathUtil.inputModulus(heading, -180, 180);

        return Rotation2d.fromDegrees(inputtedHeading);
    }

    /**
     * @return the robot's current velocity
     */
    public ChassisSpeeds getCurrentVelocity() {
        return constants.getKinematics().toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the heading of the robot.
     *
     * @param heading the new heading
     */
    public void setHeading(Rotation2d heading) {
        swerveIO.setHeading(heading);
    }

    /**
     * @return the pitch of the swerve, in degrees
     */
    public double getPitch() {
        return swerveInputs.gyroPitchDegrees;
    }

    /**
     * @return the swerve's module's positions
     */
    public SwerveModulePosition[] getModulePositions() {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getCurrentPosition();

        return swerveModulePositions;
    }

    public boolean atPose(Pose2d pose2d) {
        return atXAxisPosition(pose2d.getX()) && atYAxisPosition(pose2d.getY()) && atAngle(pose2d.getRotation());
    }

    public boolean atXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(getCurrentVelocity(), RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation()).vxMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean atYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(getCurrentVelocity(), RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation()).vyMetersPerSecond;
        return atTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    /**
     * Checks if the swerve is at an angle, within the rotation tolerance.
     *
     * @param angle the angle to check
     * @return whether the swerve is at the angle
     */
    public boolean atAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees()) < constants.getRotationTolerance() &&
                Math.abs(getCurrentVelocity().omegaRadiansPerSecond) < constants.getRotationVelocityTolerance();
    }

    /**
     * Locks the swerve, so it'll be hard to move it.
     */
    void lockSwerve() {
        setBrake(true);
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modulesIO[0].setTargetState(right);
        modulesIO[1].setTargetState(left);
        modulesIO[2].setTargetState(left);
        modulesIO[3].setTargetState(right);
    }

    /**
     * Drives the x-axis of the swerve with the given power, relative to the field's frame of reference.
     * This will also lock the y-axis and angle.
     *
     * @param xPower     the target x power
     * @param yAxisLock  the y position to lock the robot to
     * @param angleLock  the angle to lock the robot to
     * @param rateLimitX whether to rate limit the x-axis (no need for y-axis since it's locked)
     */
    void xAxisDrive(double xPower, double yAxisLock, Rotation2d angleLock, boolean rateLimitX) {
        yAxisLock = AllianceUtilities.isBlueAlliance() ? yAxisLock : FieldConstants.FIELD_WIDTH_METERS - yAxisLock;
        angleLock = AllianceUtilities.isBlueAlliance() ? angleLock : angleLock.unaryMinus();

        constants.getRotationController().setGoal(angleLock.getDegrees());
        constants.getProfiledYAxisController().setGoal(yAxisLock);

        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();
        final double currentYPosition = currentPose.getY();
        final Rotation2d currentAngle = currentPose.getRotation();
        final double yOutput = constants.getProfiledYAxisController().calculate(currentYPosition);
        final double thetaOutput = constants.getRotationController().calculate(currentAngle.getDegrees());
        profiledTargetPose = new Pose2d(
                currentPose.getX(),
                constants.getProfiledYAxisController().getSetpoint().position,
                Rotation2d.fromDegrees(constants.getRotationController().getSetpoint().position)
        );

        fieldRelativeDrive(
                new Translation2d(
                        xPower * constants.getMaxSpeedMetersPerSecond(),
                        yOutput
                ),
                Rotation2d.fromDegrees(thetaOutput),
                rateLimitX,
                false,
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation()
        );
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the target x power
     * @param yPower     the target y power
     * @param thetaPower the target theta power
     * @param rateLimit  whether the swerve should be rate limited
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower, boolean rateLimit) {
        selfRelativeDrive(
                getDriveTranslation(xPower, yPower),
                getDriveRotation(thetaPower),
                rateLimit,
                rateLimit
        );
    }

    /**
     * Drives the swerve with the given velocities, relative to the robot's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     * @param rateLimitX  whether the swerve's x-axis should be rate limited
     * @param rateLimitY  whether the swerve's y-axis should be rate limited
     */
    void selfRelativeDrive(Translation2d translation, Rotation2d rotation, boolean rateLimitX, boolean rateLimitY) {
        if (rateLimitX)
            translation = rateLimit(translation, rateLimitX, rateLimitY);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians()
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the target x power
     * @param yPower     the target y power
     * @param thetaPower the target theta power
     * @param rateLimit  whether the swerve should be rate limited
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower, boolean rateLimit) {
        fieldRelativeDrive(
                getDriveTranslation(xPower, yPower),
                getDriveRotation(thetaPower),
                rateLimit,
                rateLimit,
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation()
        );
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the target x power
     * @param yPower     the target y power
     * @param thetaPower the target theta power
     * @param rateLimit  whether the swerve should be rate limited
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower, boolean rateLimit, Rotation2d robotAngle) {
        fieldRelativeDrive(
                getDriveTranslation(xPower, yPower),
                getDriveRotation(thetaPower),
                rateLimit,
                rateLimit,
                robotAngle
        );
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower    the target x power
     * @param yPower    the target y power
     * @param angle     the target angle
     * @param rateLimit whether the swerve should be rate limited
     */
    void fieldRelativeDrive(double xPower, double yPower, Rotation2d angle, boolean rateLimit) {
        constants.getRotationController().setGoal(angle.getDegrees());
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        profiledTargetPose = new Pose2d(
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation(),
                Rotation2d.fromDegrees(constants.getRotationController().getSetpoint().position)
        );
        fieldRelativeDrive(
                getDriveTranslation(xPower, yPower),
                Rotation2d.fromDegrees(
                        constants.getRotationController().calculate(currentAngle.getDegrees())
                ),
                rateLimit,
                rateLimit,
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation()
        );
    }

    /**
     * Drives the swerve with the given velocities, relative to the field's frame of reference.
     *
     * @param translation the target x and y velocities in m/s
     * @param rotation    the target theta velocity in radians per second
     * @param rateLimitX  whether the swerve's x-axis should be rate limited
     * @param rateLimitY  whether the swerve's y-axis should be rate limited
     * @param robotAngle  the current robot angle, to drive relative from
     */
    void fieldRelativeDrive(Translation2d translation, Rotation2d rotation, boolean rateLimitX, boolean rateLimitY, Rotation2d robotAngle) {
        translation = rateLimit(translation, rateLimitX, rateLimitY);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation.getRadians(),
                robotAngle
        );
        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Resets the rotation controller of the swerve.
     */
    void resetRotationController() {
        constants.getRotationController().reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation().getDegrees());
    }

    /**
     * Resets the y-axis controller of the swerve.
     */
    void resetYAxisController() {
        constants.getProfiledYAxisController().reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose().getY());
    }

    /**
     * Sets whether the swerve drive should be in closed loop control, or in open loop control.
     *
     * @param closedLoop true if the drive motor should be in closed loop control, false if it should be in open loop control
     */
    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setDriveMotorClosedLoop(closedLoop);
    }

    /**
     * Stops the swerve's motors.
     */
    void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    /**
     * Sets whether the drive motors should brake or coast.
     *
     * @param brake whether the drive motors should brake or coast
     */
    void setBrake(boolean brake) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setBrake(brake);
    }

    /**
     * Sets the swerve's target module states.
     *
     * @param swerveModuleStates the target module states
     */
    void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    private boolean atTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < constants.getTranslationTolerance() &&
                Math.abs(currentTranslationVelocity) < constants.getTranslationVelocityTolerance();
    }

    private Rotation2d getDriveRotation(double rotPower) {
        return new Rotation2d(rotPower * constants.getMaxRotationalSpeedRadiansPerSecond());
    }

    private Translation2d getDriveTranslation(double x, double y) {
        return new Translation2d(
                x * constants.getMaxSpeedMetersPerSecond(),
                y * constants.getMaxSpeedMetersPerSecond()
        );
    }

    private Translation2d rateLimit(Translation2d toLimit, boolean rateLimitX, boolean rateLimitY) {
        return new Translation2d(
                rateLimitX ? constants.getXSlewRateLimiter().calculate(toLimit.getX()) : toLimit.getX(),
                rateLimitY ? constants.getYSlewRateLimiter().calculate(toLimit.getY()) : toLimit.getY()
        );
    }

    private void updateNetworkTables() {
        Logger.getInstance().recordOutput("Swerve/Velocity/rot", getCurrentVelocity().omegaRadiansPerSecond);
        Logger.getInstance().recordOutput("Swerve/Velocity/x", getCurrentVelocity().vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Velocity/y", getCurrentVelocity().vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/currentCommand", getCurrentCommand() == null ? "null" : getCurrentCommand().getName());
        Logger.getInstance().recordOutput("Swerve/currentStates", getModuleStates());
        Logger.getInstance().recordOutput("Swerve/targetStates", getTargetStates());

        if (RobotContainer.POSE_ESTIMATOR == null)
            return;
        if (profiledTargetPose == null) {
            Logger.getInstance().recordOutput("targetPose", RobotContainer.POSE_ESTIMATOR.getCurrentPose());
        } else {
            Logger.getInstance().recordOutput("targetPose", profiledTargetPose);
            profiledTargetPose = null;
        }
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = constants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return
                Math.abs(chassisSpeeds.vxMetersPerSecond) <= constants.getDriveNeutralDeadband() &&
                        Math.abs(chassisSpeeds.vyMetersPerSecond) <= constants.getDriveNeutralDeadband() &&
                        Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= constants.getRotationNeutralDeadband();
    }

    private SwerveConstants generateConstants() {
        switch (RobotConstants.ROBOT_TYPE) {
            case KABLAMA:
                return new TrihardSwerveConstants();
            case TRIHARD:
                return new KablamaSwerveConstants();
            default:
                return new SimulationSwerveConstants();
        }
    }

    private SwerveModuleIO[] getModulesIO() {
        if (RobotConstants.IS_REPLAY) {
            final SwerveModuleIO[] modulesIO = new SwerveModuleIO[4];
            modulesIO[0] = new SwerveModuleIO("FRONT_LEFT");
            modulesIO[1] = new SwerveModuleIO("FRONT_RIGHT");
            modulesIO[2] = new SwerveModuleIO("REAR_LEFT");
            modulesIO[3] = new SwerveModuleIO("REAR_RIGHT");
            return modulesIO;
        }

        return constants.getModulesIO();
    }

    private SwerveIO generateIO() {
        if (RobotConstants.IS_REPLAY)
            return new SwerveIO();

        switch (RobotConstants.ROBOT_TYPE) {
            case KABLAMA:
                return new KablamaSwerveIO();
            case TRIHARD:
                return new TrihardSwerveIO();
            default:
                return new SimulationSwerveIO();
        }
    }
}

