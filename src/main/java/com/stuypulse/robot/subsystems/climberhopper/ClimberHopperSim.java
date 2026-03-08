/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.subsystems.climberhopper;


import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ClimberHopperSim extends ClimberHopper {
    
    private final ElevatorSim sim;
    private final ClimberHopperVisualizer visualizer;
    private double voltage;

    private Optional<Double> voltageOverride;

    private final StructPublisher<Pose3d> hopperPublisher = NetworkTableInstance.getDefault().getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();

    public ClimberHopperSim() {
        visualizer = ClimberHopperVisualizer.getInstance();

        sim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            Settings.ClimberHopper.GEAR_RATIO,
            Settings.ClimberHopper.MASS_KG,
            Settings.ClimberHopper.DRUM_RADIUS_METERS,
            Settings.ClimberHopper.MIN_HEIGHT_METERS,
            Settings.ClimberHopper.MAX_HEIGHT_METERS,
            false,
            Settings.ClimberHopper.MIN_HEIGHT_METERS
        );

        voltageOverride = Optional.empty();
    }

    public boolean getStalling() {
        return sim.getCurrentDrawAmps() > Settings.ClimberHopper.STALL;
    }

    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public double getCurrentRotations() {
        return sim.getPositionMeters() / Settings.ClimberHopper.POSITION_CONVERSION_FACTOR;
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getState().getTargetHeight() - getCurrentHeight()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.ClimberHopper.HEIGHT_TOLERANCE_METERS);
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void resetPostionUpper() {
        // No encoder reset for sim
    }


    @Override
    public void periodic() {
        super.periodic();

        if (voltageOverride.isPresent()) {
                voltage = voltageOverride.get();
        } else { 
            if (!atTargetHeight()) {
                if (getCurrentHeight() < getState().getTargetHeight()) {
                    voltage = Settings.ClimberHopper.MOTOR_VOLTAGE;
                } else {
                    voltage = - Settings.ClimberHopper.MOTOR_VOLTAGE;
                }
            } else {
                voltage = 0;
            }
        }

        sim.setInputVoltage(voltage);

        SmartDashboard.putNumber("ClimberHopper/Voltage", voltage);
        SmartDashboard.putNumber("ClimberHopper/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putBoolean("ClimberHopper/Stalling", getStalling());
        SmartDashboard.putNumber("ClimberHopper/Height", getCurrentHeight());
        visualizer.update(getCurrentHeight());

        sim.update(0.02);

        hopperPublisher.set(new Pose3d(0, getCurrentHeight(), 0, new Rotation3d(0, 0, 0)));
    }
}