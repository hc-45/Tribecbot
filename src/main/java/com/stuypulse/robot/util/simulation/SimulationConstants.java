/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.    */
/* Use of this source code is governed by an MIT-style license    */
/* that can be found in the repository LICENSE file.              */
/*******************************************************************/

package com.stuypulse.robot.util.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public interface SimulationConstants {
    /**
     * <h2>Record that holds CAD Offsets</h2>
     * <p>Sourced from CAD exports to hold a component's positional offsets from their position in CAD to their position in sim</p>
     * <pre>{@code
     * Offsets SHOOTER = new Offsets(-0.1016, 0.2032, 0.3255,
     *     Degrees.of(90), Degrees.of(0), Degrees.of(-90));
     *
     * Pose3d shooterPose = SHOOTER.withRotation(
     *     new Rotation3d(0, 0, turretSim.getAngle().getRadians())
     * );
     * }</pre>
     * @param x X translation from the robot origin (meters)
     * @param y Y translation from the robot origin (meters)
     * @param z Z translation from the robot origin (meters)
     * @param roll Rotation about the X axis
     * @param pitch Rotation about the Y axis
     * @param yaw Rotation about the Z axis
     */
    public static record Offsets(double x, double y, double z,
        Angle roll, Angle pitch, Angle yaw) {

        /**
         * <h2>Constructs an Offsets instance with no rotation</h2>
         * @param x X translation from the robot origin (meters)
         * @param y Y translation from the robot origin (meters)
         * @param z Z translation from the robot origin (meters)
         */
        public Offsets(double x, double y, double z) {
            this(x, y, z, Radians.of(0), Radians.of(0), Radians.of(0));
        }

        /**
         * <h2>Constructs an Offsets instance with no translation</h2>
         * @param roll Rotation about the X axis
         * @param pitch Rotation about the Y axis
         * @param yaw Rotation about the Z axis
         */
        public Offsets(Angle roll, Angle pitch, Angle yaw) {
            this(0, 0, 0, roll, pitch, yaw);
        }

        /**
         * <h2>Translational component as a {@link Translation3d}</h2>
         * @return translation from the robot origin
         */
        public Translation3d toTranslation3d() {
            return new Translation3d(x, y, z);
        }

        /**
         * <h2>Rotational component as a {@link Rotation3d}</h2>
         * @return Rotation3d of the roll, pitch, and yaw components of the offset
         */
        public Rotation3d toRotation3d() {
            return new Rotation3d(roll, pitch, yaw);
        }

        /**
         * <h2>This offset as a {@link Pose3d}.</h2>
         * <p>Useful for components whose pose is fully static and requires no adjustments</p>
         * @return pose at this offset's position and orientation
         */
        public Pose3d toPose3d() {
            return new Pose3d(toTranslation3d(), toRotation3d());
        }

        /**
         * <h2>Applies this offset's translation and rotation onto an existing {@link Pose3d}</h2>
         * @param pose the base pose to offset
         * @return a new pose with this offset applied to both translation and rotation
         */
        public Pose3d applyToPose3d(Pose3d pose) {
            return new Pose3d(
                pose.getX() + x, pose.getY() + y, pose.getZ() + z,
                applyToRotation3d(pose.getRotation())
            );
        }

        /**
         * <h2>Composes this offset's rotation onto an existing {@link Rotation3d}</h2>
         * @param rotation the base rotation to offset
         * @return a new rotation with this offset's rotation applied on top
         */
        public Rotation3d applyToRotation3d(Rotation3d rotation) {
            return rotation.plus(toRotation3d());
        }

        /**
         * <h2>Pose at this offset's translation with an additional rotation</h2>
         * <p>Applies a rotation to this offsets rotation
         * <pre>{@code
         * SHOOTER_OFFSETS.withRotation(
         *     new Rotation3d(0, 0, turretSim.getAngle().getRadians())
         * );
         * }</pre>
         * @param rotation the additional rotation to add to this offset's rotation
         * @return a new pose at this offset's translation with the combined rotation
         */
        public Pose3d withRotation(Rotation3d rotation) {
            return new Pose3d(toTranslation3d(), toRotation3d().plus(rotation));
        }

        /**
         * <h2>Applies this offset's X/Y translation and yaw onto an existing {@link Pose2d}</h2>
         * <p>Roll and pitch are ignored because they have no 2D equivalent
         * @param pose the base 2D pose to offset
         * @return a new pose of this offset's X, Y, and yaw
         */
        public Pose2d applyToPose2d(Pose2d pose) {
            return new Pose2d(pose.getX() + x, pose.getY() + y,
                applyToRotation2d(pose.getRotation()));
        }

        /**
         * <h2>Applies this offset's yaw onto an existing {@link Rotation2d}</h2>
         * <p>Roll and pitch are ignored because they have no 2D equivalent
         * @param rotation the base 2D rotation to offset
         * @return a new rotation with this offset's yaw applied on top
         */
        public Rotation2d applyToRotation2d(Rotation2d rotation) {
            return rotation.plus(new Rotation2d(yaw));
        }
    }

    public interface Shooter {
        double BPS = 8.0; // balls per second, TODO: get actual value
        double COMPRESSION_FACTOR = 0.85; // how much the flywheel compresses the gamepiece and thus affects its exit speed

        public static double rpmToMps(double RPM) {
            return ((Settings.Superstructure.Shooter.FLYWHEEL_RADIUS * RPM * Math.PI) / 60.0) * COMPRESSION_FACTOR;
        }

        public Offsets OFFSETS = new Offsets(
            -0.1016,
            0.2032,
            0.3255608932,
            Degrees.of(90), // maybe 0?
            Degrees.of(0),
            Degrees.of(-90.0) // CAD zero angle offset degrees
        );
    }

    public interface Intake {
        double INTAKE_WIDTH = 0.5;
        double INTAKE_LENGTH = 0.15;

        double PIVOT_ARM_LENGTH = 0.2639822;
        double PIVOT_END_X = 0.1093388152;

        public Offsets PIVOT_OFFSETS = new Offsets(
            0.2393388152,
            0,
            0.19685, // CAD zero angle offset degrees
            Degrees.of(-19.0),
            Degrees.of(0),
            Degrees.of(90)
        );

        public Offsets ROLLER_OFFSETS = new Offsets(0.022, 0, 0.2152848, Degrees.of(90), Degrees.of(0), Degrees.of(90));
    }

    public interface Hopper {
        int FUEL_CAPACITY = 67;

        public Offsets OFFSETS = new Offsets(-0.38, 0, 0, Degrees.of(90), Degrees.of(0), Degrees.of(90));
    }

    /****************/
    /*** VARIANCE ***/
    /****************/

    // TODO: better variance values
    double LAUNCH_X_VARIANCE = 0.0;
    double LAUNCH_Y_VARIANCE = 0.0;
    double LAUNCH_YAW_VARIANCE = 7.5;
    double LAUNCH_SPEED_VARIANCE = 0.75;
    double LAUNCH_PITCH_VARIANCE = 5.0;
    
    double HUB_TARGET_Z = 1.3; // height of the hub in meters

    double HUB_TOLERANCE_XY = 0.3; // tolerance for a shot to be counted as a score, in meters
}