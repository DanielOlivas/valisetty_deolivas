/**
 * DVRobo - a robot by Daniel Olivas and Vishal Alisetty
 */
 
 package valisetty_deolivas;
import robocode.*;
import java.awt.*; // Used to import the classes present in the package java.awt
import java.awt.geom.Point2D; // Allows us to use the (X,Y) coordinate plane
import java.awt.Color;


import robocode.util.*;


public class DVRobo extends AdvancedRobot{
    
    // Velocity aim we are looking at for the opponent(s)
    static int enemyVAim;
    
    // The enemy's velocity
    static int currentEnemyV;
    
    // How much to turn our gun when searching
    double gunTurnAmount;
    
    // Name of the robot we're currently tracking
    String trackName;
    
    // number of velocities that will be stored during one time. 250 we found is the most optimal.
    static int vDepth = 250;
    
    // Data used to store enemy's every movement
    static double enemyV [][];
    
    // Determines whether or not we changed our gun's depth.
    static boolean vDepthChange = true;
    
    //Robo's average Velocity
    double aimingV;
     
    //Necessary Variables
    int averageC;
    int c;
    boolean fire;
    double time;
      
    //BulletPower
    static double bulletP;
    static double turnD;
      
    //Enemy's X and Y Values
    static double EnemyX;
    static double EnemyY;
      
    //Used to split the middle of the field to 4 quadrants     
    static double fieldXMiddle;
    static double fieldYMiddle;  
          
    // The distance required from the enemy
    int direction = 1;
    double distanceFromE = 1000;
        
    // Key component to trap enemy in corner for them to not escape
    double cornerX;
    double cornerY;
    
    // The closest distance we want to be from our enemy. We need this to push enemy robots to a corner
    static double closestDistance = 150 ;
    
    // The amount to modify our robot's angle by.
    static double fireAngleMod = 1.0;
    
    // The old heading of the enemy, used to find turn rate.         
    double oldEnemyHeading;
         
    static boolean first = true;
        
        
    public void run() {
        // Initialization of the robot should be put here

        // After trying out your robot, try uncommenting the import at the top,
        // and the next line:
        setBodyColor(Color.blue);
        setGunColor(Color.red);
        setRadarColor(Color.red);
        setScanColor(Color.blue);
        setBulletColor(Color.yellow);
    
        if(first){
            aimingV = 4;
            currentEnemyV = 1;
        }
        first = false;
 
        if(vDepthChange){
            c = 0;
            enemyV = new double[vDepth][6];
            double assumeV = aimingV;
            while(c < 6){
                if(c == 0||c == 4){
                    if(currentEnemyV == 0||currentEnemyV == 4){
                        assumeV = aimingV;
                    }
                    else{
                        assumeV =- aimingV;
                    }
                }
                  else{
                    if(currentEnemyV == 1||currentEnemyV == 3){
                        assumeV=aimingV;
                    }
                      else{
                        assumeV =- aimingV;
                    }
 
                }
                averageC = 0;
                while(averageC < vDepth){
                    enemyV[averageC][c] = assumeV;
                    averageC++;
                }
                c++;
            }
            c = 0;
        }
        vDepthChange = false;
        
        // Used to find middle of the field
        fieldXMiddle = getBattleFieldHeight()/2;
        fieldYMiddle = getBattleFieldWidth()/2;
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        do{
            turnRadarRightRadians(Double.POSITIVE_INFINITY);
        }while(true);
    }
        
    public void onScannedRobot(ScannedRobotEvent d){
        
        // The maximum bulletPower is 3, unless we are further than 250 units away from the enemy, in which case it is 2.4.
        bulletP = 3.0;
        if(d.getDistance() > 250){
                    bulletP = 2.4;
                    
       }
       
       ////Find our enemy's absolute bearing and makes a thing used for painting(That option during battle).
       double absBearing=d.getBearingRadians()+getHeadingRadians();
       Graphics2D g=getGraphics();
       
     
       double bulletPow = Math.min(bulletP, Math.min(getEnergy()/12, d.getEnergy()/4));
       double bulletSpeed = 20-4*bulletPow;

       //This finds our enemies X and Y.
       EnemyX = getX()+d.getDistance() *Math.sin(absBearing);
       EnemyY = getY()+d.getDistance() *Math.cos(absBearing);
       
       //Find out which velocity segment our enemy is at right now.
       if(d.getVelocity() < -3){
                   currentEnemyV = 0;         
         }
         else if(d.getVelocity() >3){
                   currentEnemyV = 1;         
         }
         else if(d.getVelocity() <= 3&&d.getVelocity() >= -3){
                 if(currentEnemyV == 0){
                         currentEnemyV = 3;     
                   }
         else if(currentEnemyV == 1){
                     currentEnemyV = 3;     
           }   
      }
                 
      // Used to update our aiming segment each time a little bit of time has passed for a bullet to hit them and we have fired;
      if(getTime()-time > d.getDistance()/12.8 && fire == true){
            enemyVAim = currentEnemyV;
        }
      else{
          fire=false;
      }
        
      // Records a new enemy velocity and raises the count.
      enemyV[c][enemyVAim] = d.getVelocity();
      c++;
      if(c==vDepth){
          c=0;
      }
      
      // Calculates our average velocity for the list of velocities at our current segment.
      averageC = 0;
      aimingV = 0;
      while(averageC<vDepth){
          aimingV += enemyV[averageC][currentEnemyV];
          averageC++;
      }
      aimingV/=vDepth;
      
      // Find our enemy's heading and heading change.
      double enemyHeading = d.getHeadingRadians();
      double enemyHeadingChange = enemyHeading - oldEnemyHeading;
      oldEnemyHeading = enemyHeading;
      
      // Our method of targeting will be Circular Targeting. This form of targeting assumes that our enemy will keep moving with the same speed and turn rate 
      // that they are using when firing. This could be good or bad depending on what we are coming up against.
      double deltaTiming = 0;
      double predictX = EnemyX, predictY = EnemyY;
      while((++deltaTiming) * bulletSpeed < Point2D.Double.distance(getY(), getX(), predictY, predictX)){
      
            // Adding the movement we think our enemy will make to our enemy's current X and Y
            predictY += Math.cos(enemyHeading) * aimingV;
            predictX += Math.sin(enemyHeading) * aimingV;
            
            // Find our enemy's heading changes.
            enemyHeading += enemyHeadingChange;

            // Paints the path we think our enemy will take. This helps us, by giving us feedback. 
            g.setColor(Color.red);
            g.fillOval((int)predictX-2, (int)predictY-2, 4, 4);
            
            // If our predicted coordinates are outside the walls, put them 20 units away from the walls as we know 
            // that this is the closest they can get to the wall 
            predictX = Math.max(predictX,18);
            predictY = Math.max(predictY,18);
            predictX = Math.min(predictX,getBattleFieldWidth()-18);
            predictY = Math.min(predictY,getBattleFieldHeight()-18);
     }
     
     // Find the bearing of our predicted coordinates from us.
     double aiming = Utils.normalAbsoluteAngle(Math.atan2(  predictX - getX(), predictY - getY()));
     
     //Aim and fire.
     setTurnGunRightRadians(Utils.normalRelativeAngle(aiming - getGunHeadingRadians()));
     setFire(bulletPow);
          
     // Allows the Robot to find, and trap the opponent at some corner
     if(EnemyX>fieldXMiddle){
          cornerX = getBattleFieldWidth();
      }
      else{
          cornerX = 0;
      }
      if(EnemyY>fieldYMiddle){
          cornerY = getBattleFieldHeight();
      }
      else{
          cornerY = 0;
      }
  
     // This allows to find the bearing of the enemy from the corner. This is done by using Math.    
     double bearingEnemy = Utils.normalAbsoluteAngle(Math.atan2(EnemyX-cornerX,EnemyY-cornerY));    
      
     // This allows to find the distance of the enemy from the corner. This is done by using Math.         
     double distanceEnemy = Math.sqrt(Math.pow((EnemyX-cornerX),2)+Math.pow((EnemyY-cornerY),2));
      
     // This assigns coordinates for where we want to move.
     double targetEnemyX = cornerX+(distanceEnemy+distanceFromE)*Math.sin( bearingEnemy);
     double targetEnemyY = cornerY+(distanceEnemy+distanceFromE)*Math.cos( bearingEnemy);
 
     // This is used in order aim in the field. Calculation done 
     targetEnemyX = Math.max(18, targetEnemyX);
     targetEnemyX = Math.min(getBattleFieldWidth()-20, targetEnemyX);
     targetEnemyY = Math.max(18, targetEnemyY);
     targetEnemyY = Math.min(getBattleFieldHeight()-20, targetEnemyY); 
     
     // Used to paint the target/opponent
     g.setColor(Color.blue);
     g.fillOval((int)targetEnemyX-7, (int)targetEnemyY-7, 8, 8);
     g.setColor(Color.red);
     g.fillOval((int)targetEnemyX-5, (int)targetEnemyY-5, 7, 7);
     g.setColor(Color.green);
     g.fillOval((int)targetEnemyX-8, (int)targetEnemyY-8, 6, 6);

     // Making distance closer to the Enemy/Opponent
        if(getX()>targetEnemyX-20&&getX()<targetEnemyX+20&&getY()>targetEnemyY-20&&getY()<targetEnemyY+20){
            distanceFromE -= 50;
        }
     
     // This is to prevent us from hitting our enemy.
     distanceFromE = Math.max(distanceFromE, closestDistance);
   
     // Making the maximum distance equivalent to our distance from the enemy robot.
     distanceFromE = Math.min(distanceFromE,d.getDistance());
     
     // This finds the bearing of the point from us.
     double inverseT = Utils.normalAbsoluteAngle(Math.atan2(targetEnemyX-getX(),targetEnemyY-getY()));
     
     // If we are going backwards, we want to reflect our target points over our robot for the purposes of turning.
     if(direction == -1){
            targetEnemyX = getX()-100*Math.cos(inverseT);
            targetEnemyY = getY()-100*Math.sin(inverseT);
        }

    // This finds the  bearing of our target coordinates from us.
    turnD = Utils.normalAbsoluteAngle(Math.atan2(targetEnemyX-getX(),targetEnemyY-getY())); 
    
    // This is neccesary for the robot to work the way we want it to work.
    setAhead(100*direction);
    setTurnRightRadians(robocode.util.Utils.normalRelativeAngle(turnD-getHeadingRadians()));
    setTurnRadarRightRadians(robocode.util.Utils.normalRelativeAngle(absBearing-getRadarHeadingRadians())*3);
    }
    
    //This deals with movement after killing an enemy robot
    public void onDeath(DeathEvent d){
       closestDistance = Math.max(75, 500*Math.random());
       if(Math.random()>.5){
            closestDistance=Math.max(75,150*Math.random());
       }

       vDepth=(int)(500*Math.random());
       vDepthChange=true;
    }
    
    // Just a little thing to do in the event that we win.
    public void onWin(WinEvent d) {
        
        for (int i = 0; i < 50; i++){
            back(75);
            fire(3);
            turnRight(360);
        }
    }
      
    // This method will be used for our robot to change hopefully change targets when it collides with another robot.
    // We hope this fixes a problem where our robot will lock on to an enemy robot across the field and bump into another robot along it's path and not
    // attack it, since it's right there in it's face.      
    public void onHitRobot(HitRobotEvent d) {

        if (trackName != null && !trackName.equals(d.getName())){
                out.println("Tracking " + d.getName() + " due to collision"); 
                }
            
        // Set the target
        trackName = d.getName();
    
        // Some commands for the robot when it does come in contact with an enemy robot. 
        setFire(3);
        setBack(75);
        setTurnGunRight(gunTurnAmount);
        gunTurnAmount = normalRelativeAngle(d.getBearing() + (getHeading() - getRadarHeading()));
        execute(); // This is necessary for the thing to work.
   }

    // Returns angle such that -180 < angle <= 180. Used for onHitRobot Method.
    public double normalRelativeAngle(double angle) {
        
        if (angle > -180 && angle <= 180)
            return angle;
        double fixedAngle = angle;
        while (fixedAngle <= -180)
            fixedAngle += 360;
        while (fixedAngle > 180)
            fixedAngle -= 360;
        return fixedAngle;
   }
}               
            