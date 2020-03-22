#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <std_msgs/String.h>
#include <string>

#define success false
#define running true

float scale = 4.0/100.0;

using namespace rrt;

bool status = running;

void initializeMarkers2D(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath,
    visualization_msgs::Marker &obstacle)
{
  //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = obstacle.header.frame_id = "path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = obstacle.header.stamp = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = obstacle.ns = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = obstacle.action = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = obstacle.pose.orientation.w = 1.0;

  //setting id for each marker
  sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
  finalPath.id      = 4;
  obstacle.id       = 100;

	//defining types
	rrtTreeMarker.type =  visualization_msgs::Marker::LINE_LIST;
  obstacle.type = visualization_msgs::Marker::LINE_STRIP;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

  //setting scale
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x     = 1;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
  sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
  sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;
  obstacle.scale.x = obstacle.scale.y =1;

  //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
  randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

  obstacle.color.g = 0.0f;
  obstacle.color.b = 0.0f;
  obstacle.color.r = 0.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a =  1.0f;
  obstacle.color.a  = 1.0f;
}




//2D obstacle
 vector<geometry_msgs::Point>  initializeObstacles2D()
{
    vector<geometry_msgs::Point> obstaclePosition;

    geometry_msgs::Point point;

    //first point
    point.x = 50;
    point.y = 50;
    point.z = 0;
    obstaclePosition.push_back(point);

    //second point
    point.x = 50;
    point.y = 70;
    point.z = 0;
    obstaclePosition.push_back(point);

    //third point
    point.x = 80;
    point.y = 70;
    point.z = 0;
    obstaclePosition.push_back(point);

    //fourth point
    point.x = 80;
    point.y = 50;
    point.z = 0;
    obstaclePosition.push_back(point);

    //first point again to complete the box
    point.x = 50;
    point.y = 50;
    point.z = 0;
    obstaclePosition.push_back(point);

    return obstaclePosition;
}


/*
within a radius r, finding all the existing nodes
*/
vector<geometry_msgs::Point>  findAllNodein2D(RRT &myRRT, RRT::rrtNode &tempNode, int radius,  vector<geometry_msgs::Point > &obstArray)
{
  vector<geometry_msgs::Point> nearPoint;
  geometry_msgs::Point point;
  int i, j, twoPointDistance = 9999;
  for(i = 0; i < myRRT.getTreeSize(); i++)
  {
        twoPointDistance = sqrt(pow(tempNode.posX - myRRT.getPosX(i),2) + pow(tempNode.posY - myRRT.getPosY(i),2));
        if ((twoPointDistance < radius))
        {
            point.x = myRRT.getPosX(i);
            point.y = myRRT.getPosY(i);
            nearPoint.push_back(point);
        }
  }
  return nearPoint;
}


/*
Computing the cost of all the paths including newNode
*/
int computeTheCost(RRT &myRRT, RRT::rrtNode &tempNode, vector<geometry_msgs::Point> &nearPoint)
{
  vector<float> totalLength;
  geometry_msgs::Point point, new_point, old_point;
  RRT::rrtNode node;
  vector<int> path;
  int rightPointID, nearPointID;

  rightPointID = myRRT.getID(nearPoint[0].x, nearPoint[0].y);

  if(nearPoint.size() >= 2)
  {
    for(int i = 0;i < nearPoint.size();i++)
    {
      nearPointID = myRRT.getID(nearPoint[i].x, nearPoint[i].y);
      path = myRRT.getRootToEndPath(nearPointID);
      totalLength.push_back(0);
      totalLength[i] = 0;
      if(path.size() >= 2)
      {
        for(int j = 1;j < path.size();j++)
        {
          node = myRRT.getNode(path[j - 1]);
          old_point.x = node.posX;
          old_point.y = node.posY;

          node = myRRT.getNode(path[j]);
          new_point.x = node.posX;
          new_point.y = node.posY;

          totalLength[i] = totalLength[i] + sqrt(pow(old_point.x - new_point.x, 2) + pow(old_point.y - new_point.y, 2));
        }
        node = myRRT.getNode(nearPointID);
        point.x = node.posX;
        point.y = node.posY;

        totalLength[i] = totalLength[i] + sqrt(pow(point.x - tempNode.posX, 2) + pow(point.y - tempNode.posY, 2));
      }
      if(totalLength[i] > totalLength[i - 1])
      {
        rightPointID = myRRT.getID(nearPoint[i].x, nearPoint[i].y);
      }
      else
      {
        totalLength[i] = totalLength[i - 1];
      }
    }
  }
  return rightPointID;
}


void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{
  geometry_msgs::Point point;

  point.x = tempNode.posX;
  point.y = tempNode.posY;
  rrtTreeMarker.points.push_back(point);

  RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

  point.x = parentNode.posX;
  point.y = parentNode.posY;
  rrtTreeMarker.points.push_back(point);
}


//Checking whether the tempNode is inside the 2d boundary
bool checkIfInsideBoundary2D(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0 || tempNode.posX > 100 || tempNode.posY > 100) return false;
    else
    {
      return true;
    }
}


bool checkIfOutsideObstacles2D( vector<geometry_msgs::Point> &obstArray2D, RRT::rrtNode &tempNode)
{
      if(tempNode.posX <= obstArray2D[1].x || tempNode.posX >= obstArray2D[2].x || tempNode.posY >= obstArray2D[1].y || tempNode.posY <= obstArray2D[0].y)
      {
          //std::cout<<"Outside the 2D Obstacle"<<endl;
          return true;
      }
    //std::cout<<"Inside the 2D Obstacle"<<endl;
    return false;
}


void generateTempPoint(RRT::rrtNode &tempNode)
{
    int x = rand() % 150 + 1;
    int y = rand() % 150 + 1;
    tempNode.posX = x;
    tempNode.posY = y;
}

//Adding new point to rrt in 2d environment
bool addNewPointtoRRTin2D(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector<geometry_msgs::Point> &obstArray2D, visualization_msgs::Marker &sourcePoint, visualization_msgs::Marker &goalPoint)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX, tempNode.posY);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    double theta = atan2(tempNode.posY - nearestNode.posY, tempNode.posX - nearestNode.posX);

    tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
    tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

    if(checkIfInsideBoundary2D(tempNode) && checkIfOutsideObstacles2D(obstArray2D, tempNode))
    {
        vector<geometry_msgs::Point> nearPoint;
        int rightPointID;
        nearPoint = findAllNodein2D(myRRT, tempNode, 6, obstArray2D);
        rightPointID = computeTheCost(myRRT, tempNode, nearPoint);
        tempNode.parentID = rightPointID;
        tempNode.nodeID = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
  }


bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X - tempNode.posX, 2) + pow(Y - tempNode.posY, 2));
    if(distance <= 10)
    {
        return true;
    }
    return false;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    finalpath.points.push_back(point);
}


int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	  ros::NodeHandle n;

	  //defining Publisher
	  ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

      //defining markers
      visualization_msgs::Marker sourcePoint;
      visualization_msgs::Marker goalPoint;
      visualization_msgs::Marker randomPoint;
      visualization_msgs::Marker rrtTreeMarker;
      visualization_msgs::Marker finalPath;
      visualization_msgs::Marker obstacle;

      initializeMarkers2D(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath, obstacle);

      sourcePoint.pose.position.x = 2;
      sourcePoint.pose.position.y = 2;

      goalPoint.pose.position.x = 95;
      goalPoint.pose.position.y = 95;



      //initializing rrtTree
      RRT myRRT(2.0,2.0);

      int goalX, goalY;

      goalX = goalY = 95;

      int rrtStepSize = 5;

      vector< vector<int> > rrtPaths;
      vector<int> path;
      int rrtPathLimit = 10;

      int shortestPathLength = 9999;
      int shortestPath = -1;

      RRT::rrtNode tempNode;

      obstacle.points = initializeObstacles2D();

      vector<geometry_msgs::Point> obstaclePosition;

      obstaclePosition = obstacle.points;

      bool addNodeResult = false, nodeToGoal = false;

      while(ros::ok() && status)
      {
          if(rrtPaths.size() < rrtPathLimit)
          {
              generateTempPoint(tempNode);
              //std::cout<<"tempnode generated"<<endl;

              addNodeResult = addNewPointtoRRTin2D(myRRT, tempNode, rrtStepSize, obstaclePosition,  sourcePoint,  goalPoint);
              if(addNodeResult)
              {
                //  std::cout<<"tempnode accepted"<<endl;
                  addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
                  //std::cout<<"tempnode printed"<<endl;

                  rrt_publisher.publish(rrtTreeMarker);
                  rrt_publisher.publish(sourcePoint);
                  rrt_publisher.publish(goalPoint);
                  rrt_publisher.publish(obstacle);
                
                 nodeToGoal = checkNodetoGoal(goalX, goalY, tempNode);
                  if(nodeToGoal)
                  {
                      path = myRRT.getRootToEndPath(tempNode.nodeID);
                      rrtPaths.push_back(path);
                      std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                      //std::cout<<"got Root Path"<<endl;
                  }
              }
          }
          else if(rrtPaths.size() >= rrtPathLimit)
          {
              status = success;
              std::cout<<"Finding Optimal Path"<<endl;
              for(int i=0; i<rrtPaths.size();i++)
              {
                  if(rrtPaths[i].size() < shortestPath)
                  {
                      shortestPath = i;
                      shortestPathLength = rrtPaths[i].size();
                  }
              }
              setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY);
              rrt_publisher.publish(finalPath);
          }
          ros::spinOnce();
          ros::Duration(0.01).sleep();
      }
    return 1;
    }
