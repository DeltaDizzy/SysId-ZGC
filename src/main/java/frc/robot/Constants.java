// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/** Add your docs here. */
public class Constants {
    public static final class DriveConstants {

        public static final int Left_Front_ID = 1;
        public static final int Right_Front_ID = 3;
        public static final int Left_Back_ID = 2;
        public static final int Right_Back_ID = 4;

        public static final int TalonFXCountsPerRev = 2048;

        // 1 motor rev = 2048 ticks
        // gearRatio motor revs = 1 wheel rev
        // 1 wheel rev = 1 wheel circumference travelled
        // 1 wheel circumference = pi*wheel diameter

        public static final double gearRatioLow = 26.0;
        public static final double gearRatioHigh = 10.71;
        // 0.1524
        public static final double wheelDiameterMeters = Units.inchesToMeters(6);

        public static final double distancePerTickMetersLowGear = (Math.PI * wheelDiameterMeters) / (2048 * gearRatioLow);
        public static final double distancePerTickMetersHighGear = (Math.PI * wheelDiameterMeters) / (2048 * gearRatioHigh);

        public static final double trackwidthMeters = Units.inchesToMeters(28.5);       
        public static final int Shifter_Forward_Channel = 0;
        public static final int Shifter_Reverse_Channel = 0;
    }

    public static final class ArmConstants{
        public static final int ArmGearRatio = 100;
        public static final int IntakeSpark_ID = 6;
        public static final int ArmPDHPortID = 7;
        public static final int xAxisMotorID = 5;
        public static final int yAxisMotorID = 7;
        public static final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    }
}
