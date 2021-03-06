// Naive distance 
// Distance we have
// Traveling Salesman Distance/Brute Force Solution

import java.util.Random;
import java.util.Scanner;

public class ShortestPathTest {

  public static int NULLVALUE = -1;
  public static int MAXVALUE = 9999999;

  public static void main(String[] args) {


/*
    Random rand = new Random();

    //fill point array with 13 random points
    Point[] allPoints = new Point[13];

    allPoints[0] = new Point(0,0,1);
    for(int i = 1; i < allPoints.length; i++) {
      int x = rand.nextInt(11) - 5;
      int y = rand.nextInt(13) - 6;
      allPoints[i] = new Point(x, y, i+1);
    }
*/

    Point[] allPoints = new Point[13];
    allPoints[0] = new Point(0,0,1);

    System.out.println("Enter all of your points one at a time in this format: X Y\n End program with a 888");
    Scanner s = new Scanner(System.in);

    String x = s.next();
    int k = 1;
    while(k < allPoints.length) {
      String y = s.next();
      allPoints[k] = new Point(Integer.parseInt(x), Integer.parseInt(y), k+1);
      k++;
      x = s.next();
    }



    //System.out.print("CALCULATING NAIVE DISTANCE SOLUTION: ");
    int displayDistance = 0;
    for(int i = 0 ; i < allPoints.length - 1 ; i++) {
      displayDistance += pythagoreanDistance(allPoints[i], allPoints[i+1])/1000;
    }
    //System.out.println(displayDistance);

    //CREATE reference distance 2-D array
    int[][] distancesBetweenAllPoints = new int[13][13];
    for(int i = 0; i < distancesBetweenAllPoints.length ; i++) {
      for(int j = 0 ; j < distancesBetweenAllPoints[0].length ; j++) {
        distancesBetweenAllPoints[i][j] = pythagoreanDistance(allPoints[i], allPoints[j]);
      }
    }

    //DEBUG SETUP
    System.out.println("\nAll points: \n"); 
    for(int i = 0; i < allPoints.length ; i++) {
      System.out.print("(" + allPoints[i].getX() + "," + allPoints[i].getY() + ")" + " ");
    }
    //System.out.println("\n\nDistance Matrix: ");
    for(int i = 0; i < distancesBetweenAllPoints.length ; i++) {
      //System.out.println("\n");
      for(int j = 0 ; j < distancesBetweenAllPoints[0].length ; j++) {
        //System.out.print(distancesBetweenAllPoints[i][j] + " ");
      }
    }

    //we will assume that our starting value will be the first value entered in the allPoints array

    Point[] orderPoints = new Point[13];
    int currentIndex = 0;
    Point currentPoint = allPoints[currentIndex];
    orderPoints[0] = new Point(allPoints[currentIndex].getX(), allPoints[currentIndex].getY(), 1);
    allPoints[0] = null;
    int orderPointsIndex = 1;

    int nextIndex = -1;
    boolean finished = false;

    int currentMinDistance = MAXVALUE;

    while(!finished) {
      finished = true;
      nextIndex = -1;
      currentMinDistance = MAXVALUE;

      for(int i = 0; i < allPoints.length; i++) {

        if(allPoints[i] != null) {
          if(distancesBetweenAllPoints[currentIndex][i] < currentMinDistance) {
            finished = false;
            currentMinDistance = distancesBetweenAllPoints[currentIndex][i];
            nextIndex = i;
          }
        }

      }
      if(nextIndex != -1) {
        orderPoints[orderPointsIndex++] = new Point(allPoints[nextIndex].getX(), allPoints[nextIndex].getY(), allPoints[nextIndex].getOrder());
        allPoints[nextIndex] = null;
        currentIndex = nextIndex;
      }
    }

    System.out.println("\nStarting with POINT no. 1 Ordered points: \n");   
    for(int i = 0; i < orderPoints.length ; i++) {
      if(orderPoints[i] != null) {
        int val = orderPoints[i].getOrder() -1;
        System.out.println("POINT" + i + "X: " + orderPoints[i].getX());
        System.out.println("POINT" + i + "Y: " + orderPoints[i].getY());
        System.out.println("POINT" + i + "R: " + val + "\n");

      } else {
        System.out.println(" NULL, ");
      }
    }


    //System.out.print("CALCULATING OUR CALCULATED DISTANCE SOLUTION: ");
    int greedydisplayDistance = 0;
    for(int i = 0 ; i < orderPoints.length - 1 ; i++) {
      greedydisplayDistance += pythagoreanDistance(orderPoints[i], orderPoints[i+1])/1000;
    }
    //System.out.println(greedydisplayDistance);

    System.out.println("TOTAL DISTANCE: " + greedydisplayDistance);

  }

  //this function is not needed unless you cannot use null!
  public static void removeAllDistancesAtIndex(int[][] someArray, int counter) {
    for(int i = 0; i < someArray.length; i++) {
      someArray[i][counter] = MAXVALUE;
      someArray[counter][i] = MAXVALUE;
    }
  }

  /*
     return pythagorean distance between two points
   */
  public static int pythagoreanDistance(Point a, Point b) {
    if (a == b) 
      return 0;
    int value = (int) (1000 * Math.sqrt(Math.pow(a.getX()-b.getX(), 2) + Math.pow(a.getY()-b.getY(), 2)));
    return value;
  }


  private static class Point {
    private int x;
    private int y;
    private int order;

    public Point(int x, int y, int order) {
      this.x = x;
      this.y = y;
      this.order = order;
    }

    public int getX() {
      return x;
    }

    public int getY() {
      return y;
    }

    public int getOrder() {
      return order;
    }
  }

}
