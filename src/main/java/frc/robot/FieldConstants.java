package frc.robot;

/** Class for mathematic functions */
public class FieldConstants {

    /** Heights of vision camera and targets, in inches */
    public static class VisionConstants {
        static final double targetHeight = 90.875;
        static final double cameraHeight = 38;
        static final double mountingAngle = 26.4;

        public static double getTargetheight() {
            return targetHeight;
        }

        public static double getCameraheight() {
            return cameraHeight;
        }

        public static double getMountingAngle() {
            return mountingAngle;
        }
    }

    public static class GamePieces {
        static final double ballRadius = 3.5; //in inches

        public static double getBallradius() {
            return ballRadius;
        }
    }

    public static double hangHeight = 5;

    public static int numberOfTimesToRotatePanelManipulator = 6;

}