package frc.robot;

/** Class for mathematic functions */
public class Trig238 {

    /** @return linear distance (inches) from camera to target 
    * @param height (inches) between camera and center of target
    * @param angle (degrees) formed between camera and center of target
    */
    public static double calculateDistance(double height, double angle){
        double theta = FieldConstants.VisionConstants.getMountingAngle() + angle;
        double radians = Math.toRadians(theta);
        double tanRadians = Math.tan(radians);
        double distance = height / tanRadians;
        return distance;
    }

    /*Angle is in RADIANS, NOT DEGREES.
    Make sure all other values are in the same unit, meaning no inches paired with meters.
    height is height of target relaive to when ball leaves the shooter. 
    gravity is acceleration due to gravity.
    angle is the angle of the shooter.
    distance is the horizontal distance to the target. Trust that the math works.
    */
    public static double calculateBallVelocity(double height, double gravity, double angle, double distance){
        double velocity = distance / (Math.cos(angle));
        double sqrtPortion = distance*Math.tan(angle);
        sqrtPortion = sqrtPortion - height;
        sqrtPortion = 2*sqrtPortion;
        sqrtPortion = gravity / sqrtPortion;
        sqrtPortion = Math.sqrt(sqrtPortion);
        velocity = velocity*sqrtPortion;
        return velocity;

    }

    // This assumes that the ball is not sliding against the wheel nor the outside edge of the shooter
    // Make sure all units are consistent, as in no measuring one value in inches and another in centimeters
    //ballRadius is COMPRESSED ball radius
    public static double calculateSingleWheelShooterVelocity(double ballVelocity, double wheelRadius, double ballRadius){
        double multiplier = 2*ballVelocity;
        //double multiplier = 2*wheelRadius + 3*ballRadius;
        //multiplier = multiplier / (wheelRadius + ballRadius);
        double wheelVelocity = ballVelocity * multiplier;
        return wheelVelocity;
    }

    //assuming you measure velocity in units per second
    //Make sure all units are consistent, no inches with feet etc.
    public static double calculateWheelRPM(double wheelVelocity, double wheelRadius){
        double wheelcircumference = 2*wheelRadius*Math.PI;
        double wheelRPM = 60 * (wheelVelocity / wheelcircumference);
        return wheelRPM;
    }
}