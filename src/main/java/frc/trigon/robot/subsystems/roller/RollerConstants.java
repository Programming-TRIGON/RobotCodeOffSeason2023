package frc.trigon.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RollerConstants {
    public static final double ROLLER_LENGTH = 0.45;
    static final double
            OPENING_POWER = 0.2,
            CLOSING_POWER = -0.5,
            COLLECTING_POWER = -1;

    static final double ROLLER_YAW = Units.degreesToRadians(90);
    private static final double
            ROLLER_X = 0,
            ROLLER_Y = 0.25,
            ROLLER_Z = 0.277;
    static final Translation3d ROLLER_TRANSLATION = new Translation3d(
            ROLLER_X, ROLLER_Y, ROLLER_Z
    );

    static final Mechanism2d ROLLER_MECHANISM = new Mechanism2d(ROLLER_LENGTH * 2, ROLLER_LENGTH * 2, new Color8Bit(Color.kBlack));
    private static final MechanismRoot2d ROLLER_MECHANISM_ROOT = ROLLER_MECHANISM.getRoot("RollerRoot", ROLLER_LENGTH, ROLLER_LENGTH);
    static final MechanismLigament2d
            TARGET_ROLLER_LIGAMENT = ROLLER_MECHANISM_ROOT.append(new MechanismLigament2d("targetRollerLigament", ROLLER_LENGTH, 0, 10, new Color8Bit(Color.kGray))),
            ROLLER_LIGAMENT = ROLLER_MECHANISM_ROOT.append(new MechanismLigament2d("zShowfirst rollerLigament", ROLLER_LENGTH, 0, 10, new Color8Bit(Color.kBlue)));
}
