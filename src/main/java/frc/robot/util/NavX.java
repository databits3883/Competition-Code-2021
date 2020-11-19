package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.math.Rotation2;

public class NavX {
    private final AHRS navX;
    private Rotation2 adjustmentAngle = Rotation2.ZERO;
    private boolean inverted;

    public NavX(I2C.Port port) {
        this(port, (byte) 200);
    }

    public NavX(I2C.Port port, byte updateRate) {
        navX = new AHRS(port, updateRate);
    }

    public void calibrate() {
        navX.reset();
    }

    public Rotation2 getUnadjustedAngle() {
        return Rotation2.fromRadians(getAxis(Axis.YAW));
    }

    public final Rotation2 getAngle() {
		Rotation2 angle = getUnadjustedAngle().rotateBy(adjustmentAngle.inverse());

		if (inverted) {
			return angle.inverse();
		}

		return angle;
	}

    public final void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

    public double getUnadjustedRate() {
        return Math.toRadians(navX.getRate());
    }

    public double getAxis(Axis axis) {
        switch (axis) {
            case PITCH:
                return Math.toRadians(navX.getPitch());
            case ROLL:
                return Math.toRadians(navX.getRoll());
            case YAW:
                return Math.toRadians(navX.getYaw());
            default:
                return 0.0;
        }
    }

    public enum Axis {
        PITCH,
        ROLL,
        YAW
    }
}