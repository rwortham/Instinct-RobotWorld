#pragma once
#include "Instinct.h"
#include <Windows.h>
// define the available robot actions
#define ACTION_SETSPEED 1
#define ACTION_MOVEBY 2
#define ACTION_TURN 3
#define ACTION_STOP 4
#define ACTION_SLEEP 5
#define ACTION_FAIL 6
#define ACTION_SUCCEED 7
#define ACTION_MATE 8
#define ACTION_RANDOM_TURN 9

// define the available robot senses
// these return distances to obstacles in units 0-3
#define SENSE_FRONT_RIGHT 1
#define SENSE_FRONT_LEFT 2
#define SENSE_REAR_LEFT 3
#define SENSE_REAR_RIGHT 4
#define SENSE_FRONT 5
#define SENSE_REAR 6
#define SENSE_RANDOM 7
#define SENSE_FIFTY 8
#define SENSE_MATE 9

#define REMEMBERED_MATE_INTERVALS 3
#define INITIAL_MATE_INTERVAL 0x7FFF

// Senses for the Monitor Plan
#define SENSE_AVG_MATING_INTERVAL 1

// Actions for the Monitor Plan
#define ACTION_SET_MATE_PRIORITY 1
#define ACTION_RESET_MATE_PRIORITY 2
#define ACTION_MONITOR_SLEEP 3

class InstinctWorld; // resolve circular dependency
class InstinctRobot;

// this class is used to provide a 2nd set of callback functions for the Monitor Plan
class MonitorPlanWorld : public Instinct::Senses, public Instinct::Actions, public Instinct::Monitor {
public:
	MonitorPlanWorld(InstinctRobot *pInstinctRobot, Instinct::CmdPlanner *pPlan, const char *pMonLogFileName, const unsigned int uiNamesBufferSize);
	MonitorPlanWorld::~MonitorPlanWorld();
	unsigned char runPlan(void);
	int readSense(const Instinct::senseID nSense);
	unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);
	unsigned char nodeExecuted(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeSuccess(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeInProgress(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeFail(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeError(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue);
	Instinct::CmdPlanner * getPlan(void);
	Instinct::Names * getNames(void);
	SOCKET getConnectSocket(void);
	void setConnectSocket(SOCKET socket);
	void setHostParams(char *pHostName, char *pPort);
	char * getHostName(void);
	char * getPort(void);

private:
	unsigned char sleep(const int nSleepCount, const unsigned char bCheckForComplete); // stop and wait for SleepCount
	InstinctRobot *_pRobot;
	Instinct::CmdPlanner *_pParentPlan;
	int _sleepRemaining;

	Instinct::CmdPlanner *_pPlan;
	Instinct::Names *_pNames;
	char _szMonLogFileName[MAX_PATH];
	Instinct::instinctID _storedMateDrivePriority;
	char _storedRobotChar;
	char _szHostName[255];
	char _szPort[10];
	SOCKET _connectSocket;
};

// This is a very simple test robot. It includes some simple senses and an instinct plan
// The robot moves around in a world provided by an instance of InstinctWorld
// The robot implemnts the Senses and Actions interfaces that the Instinct planner needs
class InstinctRobot : public Instinct::Senses, public Instinct::Actions, public Instinct::Monitor
{
public:
	InstinctRobot(InstinctWorld *pWorld, const char cRobot, const char *pLogFileName, const char *pMonLogFileName, const unsigned int uiNamesBufferSize);
	InstinctRobot(InstinctWorld *pWorld, const char cRobot, const char *pLogFileName, const unsigned int uiNamesBufferSize) : InstinctRobot(pWorld, cRobot, pLogFileName, NULL, uiNamesBufferSize) {}
	InstinctRobot(InstinctWorld *pWorld, const char cRobot, const unsigned int uiNamesBufferSize) : InstinctRobot(pWorld, cRobot, NULL, uiNamesBufferSize) {}
	~InstinctRobot();
	unsigned char runPlan(void);

	// these are the callbacks to interface with the planner
	int readSense(const Instinct::senseID nSense);
	unsigned char executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete);
	// these are the callbacks to interface with the plan Monitor
	unsigned char nodeExecuted(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeSuccess(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeInProgress(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeFail(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeError(const Instinct::PlanNode *pPlanNode);
	unsigned char nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue);
	unsigned char writeLogFile(const Instinct::PlanNode * pPlanNode, const char *pType, const Instinct::ReleaserType *pReleaser, const int nSenseValue, const char *pLogFileName, Instinct::Names *pNames);
	unsigned char writeInstinctServer(const Instinct::PlanNode * pPlanNode, const char *pType, const Instinct::ReleaserType *pReleaser, const int nSenseValue, const unsigned char bMonitorPlan);
	Instinct::CmdPlanner * getPlan(void);
	Instinct::Names * getNames(void);
	MonitorPlanWorld * getMonitorPlanWorld(void);
	void setRobotChar(const char cChar);
	char getRobotChar(void);
	void initMateIntervals(void);
	unsigned int matingAverage(void);
	unsigned long getMatings(void);
	void resetMatings(void);
	void setInstinctServerParams(char *pHostName, char *pPort, char *pHostName2, char *pPort2);

private:
	InstinctWorld *_pWorld;
	Instinct::CmdPlanner *_pPlan;
	Instinct::Names *_pNames;
	MonitorPlanWorld *_pMonitorPlanWorld;

	char _szLogFileName[MAX_PATH];
	char _szHostName[255];
	char _szPort[10];
	SOCKET _connectSocket;
	FILETIME sStartTime;
	unsigned int _uiMateInterval;
	int _nIntervalQueue = 0;
	unsigned int _uiMateIntervals[REMEMBERED_MATE_INTERVALS];
	unsigned long _ulMatings;
};



