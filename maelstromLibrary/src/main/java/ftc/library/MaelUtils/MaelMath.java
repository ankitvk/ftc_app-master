package ftc.library.MaelUtils;


import android.graphics.Point;

import java.util.ArrayList;

import ftc.library.MaelControl.PurePursuit.MaelPose;

public class MaelMath {

    public static double calculateDistance(MaelPose point1, MaelPose point2){
        return Math.hypot(point2.x - point1.x,point2.y - point1.y);
    }

    public static double calculateDistance(double x1, double y1, double x2, double y2){
        return Math.hypot(x2 - x1,y2- y1);
    }

    public static double anglewrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

   public static ArrayList<MaelPose> lineCircleIntersection(MaelPose circleCenter, double radius,
                                                         MaelPose linePoint1, MaelPose linePoint2){

       if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
       }
       if(Math.abs(linePoint1.x - linePoint1.x) < 0.003){
           linePoint1.x = linePoint2.x + 0.003;
       }

       double m1 = (linePoint2.y  - linePoint1.y)/(linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + Math.pow(m1,2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2)*x1);

        double quadraticC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0*y1*m1*x1) + Math.pow(y1,2) - Math.pow(radius,2);

        ArrayList<MaelPose> allPoints  = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/ (2.0*quadraticA);
            //double yRoot1 = m1 * (xRoot1 - x1) + y1;
            double yRoot1 = pointSlopeForm(1,(xRoot1 - x1),y1);

            //put back the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            //double minY = linePoint1.y < linePoint2.y ? linePoint1.y : linePoint2.y;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new MaelPose(xRoot1,yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/ (2.0*quadraticA);

            double yRoot2 = pointSlopeForm(1,(xRoot1 - x1),y1);

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new MaelPose(xRoot2,yRoot2));
            }
        } catch (Exception e){

        }
        return allPoints;
   }

   public static double quadraticFormula(double A, double B, double C){
       return (-B + Math.sqrt(Math.pow(B,2) - (4.0 * A * C)))/ (2.0*A);
   }

   public static double pointSlopeForm(double slope, double x, double yIntercept){
       return slope * x + yIntercept;
   }





}
