import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

import java.util.ArrayList;

/**
 * Created by Rezoan on 21.10.2017.
 */
class Point{
    double distanceToObject;
    double distanceTravelled;


    public Point(double distanceToObject, double distanceTravelled){
        this.distanceToObject = distanceToObject;
        this.distanceTravelled = distanceTravelled;
    }
}
public class Main {
    public boolean adjustedToSurface;
    public ArrayList<Point> points = new ArrayList<>();
    public float[] distanceSample;
    public EV3GyroSensor gyroSensor;
    public EV3UltrasonicSensor frontSensor;
    public EV3UltrasonicSensor ultraSonicSensor;
    public EV3LargeRegulatedMotor motorRight;
    public EV3LargeRegulatedMotor motorLeft;
    public SampleProvider distanceProvider;
    public EV3MediumRegulatedMotor USSensorMotor;
    public int safetyDistance;
    public MovePilot ev3;
    public static void main (String [] args){
        new Main();
    }
    public Main (){
        adjustToSurface();
        ultraSonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        gyroSensor = new EV3GyroSensor(SensorPort.S2);
        frontSensor = new EV3UltrasonicSensor(SensorPort.S3);
        motorLeft = new EV3LargeRegulatedMotor(MotorPort.A);
        motorRight = new EV3LargeRegulatedMotor(MotorPort.B);
        USSensorMotor = new EV3MediumRegulatedMotor(MotorPort.C);
        safetyDistance = 10;

        adjustedToSurface = false;

        //Creates a differential type chassis with two wheels, and parses this on to
        //a MovePilot that controls the ev3.
        Wheel rightWheel = WheeledChassis.modelWheel(motorRight, 70).offset(72);
        Wheel leftWHeel = WheeledChassis.modelWheel(motorLeft, 70).offset(-72);
        Chassis chassis = new WheeledChassis(new Wheel[]{leftWHeel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
        ev3 = new MovePilot(chassis);

        initialProcedure();
    }
    public void initialProcedure(){
        // Assigns the sampleProvider called distanceProvider to the ultraSonicSensor at the top of the robot,
        //and sets the sensor in distanceMode.
        distanceProvider = ultraSonicSensor.getDistanceMode();
        distanceSample = new float[distanceProvider.sampleSize()];

        Delay.msDelay(1000);
        //Rotates the sensor so that it is facing 90 degrees to the right, takes a reading from the sensor,
        //and puts it in index 0 of the array called distanceSample. Then saves that value in the double called distanceRight.
        USSensorMotor.rotateTo(-90);
        Delay.msDelay(250);
        distanceProvider.fetchSample(distanceSample,0);
        double distanceRight = distanceSample[0];

        Delay.msDelay(1000);
        //Rotates the sensor so that it is facing 90 degrees to the left, takes a reading from the sensor,
        //and puts it in index 0 of the array called distanceSample. Then saves that value in the double called distanceLeft.
        USSensorMotor.rotateTo(90);
        Delay.msDelay(250);
        distanceProvider.fetchSample(distanceSample,0);
        double distanceLeft = distanceSample[0];
        //Reassigns the distanceProvider to the UltraSonicSensor at the front, fetches a sample,
        //and puts it into index 0 of the distanceSample array. Then saves that value in the double distanceFront.
        distanceProvider = frontSensor.getDistanceMode();
        distanceProvider.fetchSample(distanceSample, 0);
        double distanceFront = distanceSample[0];

        Delay.msDelay(250);

        USSensorMotor.rotateTo(0);
        //This part checks the nearest surface. If only one of the sides has a detectable surface, the robot will
        //travel towards that surface. If it does not have any detectable surfaces on either side, it will check if
        //there is a surface in front.
        //If none of the sensors detect a surface, it will turn right and drive forwards until it detects a surface.
        //If surfaces have been detected on both the left and the right hand side, it will travel towards the
        //surface that is closest.

        // 255 is a placeholder value. In its place should be the value returned if
        // no objects are found within the detectable zone
        if (distanceLeft==255 && distanceRight != 255){
            ev3.rotate(-90);
            USSensorMotor.rotateTo(-90);
            // move forward

        }
        else if (distanceRight == 255 && distanceLeft != 255){
            ev3.rotate(90);
            USSensorMotor.rotateTo(90);
            // move forward
        }
        else if (distanceRight == 255 && distanceLeft ==255){
            if (distanceFront != 255){
                ev3.rotate(-90);
                USSensorMotor.rotateTo(-90);
                // move forward
            }
            else {
                ev3.rotate(-90);
                USSensorMotor.rotateTo(-90);
            }
        }
        else {
            if (distanceRight<distanceLeft){
                ev3.rotate(-90);
                USSensorMotor.rotateTo(-90);

            }
            else if (distanceRight>distanceLeft) {
                ev3.rotate(90);
                USSensorMotor.rotateTo(90);
            }
            else if (distanceRight==distanceLeft) {
                ev3.rotate(-90);
                USSensorMotor.rotateTo(-90);
            }
        }

    }

    public double CalculateAngle(Point point1, Point point2){
        boolean facingTowardsSurface;
        double hypotenus = point2.distanceTravelled - point1.distanceTravelled;
        System.out.println(hypotenus);
        double k1;

        if (point2.distanceToObject==point1.distanceToObject) {
            return 0;
        }
        else if (point2.distanceToObject < point1.distanceToObject){
            k1 = point1.distanceToObject-point2.distanceToObject;
            System.out.println(k1);
            facingTowardsSurface = true;
            System.out.println(facingTowardsSurface);
        }
        else if (point2.distanceToObject > point1.distanceToObject){
            k1 = point2.distanceToObject - point1.distanceToObject;
            System.out.println(k1);
            facingTowardsSurface = false;
            System.out.println(facingTowardsSurface);
        }
        else {
            return -1;
        }
        double k2 = Math.sqrt((hypotenus*hypotenus) - (k1*k1));
        if (k2==k1){
            return 45;
        }
        double k1Dhypotenus = k1/hypotenus;
        double angle = Math.asin(k1Dhypotenus);
        angle = Math.toDegrees(angle);
        // If the robot is facing away the surface, it will have to turn right.
        // In order to turn right, the ange must be negative.
        if (!facingTowardsSurface){
            return -angle;
        }
        return angle;
    }
    public void CreatePoint(){
        //checks if the sensor actually exists
        if (ultraSonicSensor == null){
            System.out.println("No sensor found");
        } else {
            distanceProvider = ultraSonicSensor.getDistanceMode();
            distanceSample = new float[distanceProvider.sampleSize()];
            distanceProvider.fetchSample(distanceSample,0);
            double distanceTravelled = motorLeft.getTachoCount()/360 * 10;
            points.add(new Point(distanceSample[0], distanceTravelled));
        }

    }
    public void adjustToSurface(){
        //get the arrayList size in order to find out how many places are occupied
        int originalSize = points.size();
        //creates one point, waits 1 second, creates another point
        CreatePoint();
        Delay.msDelay(1000);
        CreatePoint();
       // points.add(new Point(0.19, 0.34));
       // points.add(new Point(0.11, 0.46));
        //create two new points
        //Passes on the two last recorded points in the ArrayList points, and
        // passes them on to the CalculateAngle method
        if (!points.isEmpty()){
            System.out.println(CalculateAngle(points.get(originalSize), points.get(originalSize+1)));
            ev3.rotate((int) CalculateAngle(points.get(originalSize), points.get(originalSize+1)));
            adjustedToSurface = true;
        } else {
            System.out.println("No points have been recorded");
        }

        }
    }

