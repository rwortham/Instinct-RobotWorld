#pragma once
#include <list>
#include "Instinct.h"

#define WORLD_X 90
#define WORLD_Y 40

class InstinctRobot; // resolve circular dependencies
class InstinctWorld;

// This class stores the robot and its location, orientation and speed within the InstinctWorld
// it also implements the actual work to fulfil the robots actions in the world
class InstinctWorldRobot
{
public:
	InstinctWorldRobot(InstinctWorld *pWorld, InstinctRobot *pMyRobot, const unsigned int nXpos, const unsigned int nYpos, const int nHeading, char cRobot);
	~InstinctWorldRobot();
	bool operator == (InstinctWorldRobot const & rhs);
	void operator = (InstinctWorldRobot const & rhs);

	// called by the world every cycle
	unsigned char move(unsigned char *); // calculates the new robot position based on heading, speed and so on

	// called by the robot actions
	unsigned char turn(const int nDegrees, const unsigned char bCheckForComplete); // define the new heading
	unsigned char randomTurn(const int nDegrees, const unsigned char bCheckForComplete); // define the new heading
	unsigned char setSpeed(const int nSpeed, const unsigned char bCheckForComplete); // speed in units per cycle
	unsigned char moveBy(const int nDistance, const unsigned char bCheckForComplete); // allows robot to define how far it moves (since it can use its own odometry)
	unsigned char stop(const int nActionValue, const unsigned char bCheckForComplete); // stop the robot now
	unsigned char sleep(const int nSleepCount, const unsigned char bCheckForComplete); // stop and wait for SleepCount
	unsigned char mate(const int nSleepCount, const unsigned char bCheckForComplete); // mate with conspecific if available
	void setRobotChar(const char cChar);
	char getRobotChar(void);
	int getSpeed(void);

	unsigned int nXpos; // x,y position of robot
	unsigned int nYpos;
	char robotChar; // the char representing the robot;
	InstinctRobot *pRobot;

protected:
	InstinctWorld *_pWorld;
	int _nHeading; // the robots heading in degrees (0=North=Up)
	int _newHeading; // used during a turn
	int _nSpeed;
	int _moveRemaining;
	int _sleepRemaining;
	unsigned char _turnInProgress;
	unsigned char _moveInProgress;

};



// This class provides a world for the Instinct robot. The robot can execute actions on the world, and sense the world by querying it.
// The world supports many robots.
// The world has a 2D map and stores the location of the robot(s)
class InstinctWorld
{
public:
	InstinctWorld();
	~InstinctWorld();
	void tick(void); // increment time
	void addRobot(InstinctRobot *pRobot, const unsigned int nXpos, const unsigned int nYpos, const int nHeading, const char cRobot);
	void removeRobot(InstinctRobot *pRobot);
	int readSense(InstinctRobot *pRobot, const Instinct::senseID nSense);
	unsigned char executeAction(InstinctRobot *pRobot, const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);
	char getRobotChar(InstinctRobot *pRobot);
	void setRobotChar(InstinctRobot *pRobot, const char cChar);
	void showWorld(void);
	void showWorld(wchar_t *pBuffer, const unsigned int nBuffLen);
	unsigned int totalRobots(void);
	unsigned int movingRobots(void);
	InstinctRobot * otherExists(InstinctWorldRobot *pRobot);
	unsigned char cMap[WORLD_Y][WORLD_X]; // this is the 2D world

private:
	virtual void initialiseWorld(void); // set up the map
	int readCornerSensors(InstinctRobot *pRobot, const Instinct::senseID nSense);
	int senseRandom(InstinctRobot *pRobot); // just get the world to return a random number 0-9
	int senseMate(InstinctRobot *pRobot);
	std::list <InstinctWorldRobot> _robots;

};

