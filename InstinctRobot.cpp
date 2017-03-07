// A Robot that can operate in InstinctWorld, using the Instinct Planner
#include "stdafx.h"
#include "Instinct.h"
#include "InstinctRobot.h"
#include "InstinctWorld.h"
#include <fstream>


const char *pNodeTypeNames[INSTINCT_NODE_TYPES] =
{	{ "AP" },
	{ "APE" },
	{ "C" },
	{ "CE" },
	{ "D" },
	{ "A" },
};

InstinctRobot::InstinctRobot(InstinctWorld *pWorld, const char cRobot, const char *pLogFileName, const char *pMonLogFileName, const unsigned int uiNamesBufferSize)
{
	_szLogFileName[0] = 0;
	_uiMateInterval = 0;
	_nIntervalQueue = 0;
	_ulMatings = 0;
	initMateIntervals();

	if (pLogFileName)
	{
		strncpy_s(_szLogFileName, pLogFileName, sizeof(_szLogFileName)-1);
	}

	// create a plan with zero size to start with
	Instinct::instinctID nPlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
	_pPlan = new Instinct::CmdPlanner(nPlanSize, this, this, this);
	_pNames = new Instinct::Names(uiNamesBufferSize);
	// We need a separate Monitor class to run the monitor plan and receive the callbacks 
	_pMonitorPlanWorld = new MonitorPlanWorld(this, _pPlan, pMonLogFileName, uiNamesBufferSize);
	_pWorld = pWorld; // remember the world that the robot lives in
	_pWorld->addRobot(this, WORLD_X / 2, WORLD_Y / 2, 0, cRobot); // add the robot to the middle of the world
}

void InstinctRobot::initMateIntervals(void)
{
	for (int i = 0; i < REMEMBERED_MATE_INTERVALS; i++)
	{
		_uiMateIntervals[i] = INITIAL_MATE_INTERVAL;
	}
}
// when we destroy the robot, also destroy its plan
InstinctRobot::~InstinctRobot()
{
	delete _pPlan;
	delete _pNames;
	delete _pMonitorPlanWorld;
}

// run the robots planner. This is its source of CPU resource 
unsigned char InstinctRobot::runPlan(void)
{
	unsigned char bRtn;
	_pPlan->processTimers(1);
	bRtn = _pPlan->runPlan();
	_pMonitorPlanWorld->runPlan();
	//_pWorld->setRobotChar(this, '0' + INSTINCT_RTN_DATA(bRtn)); // enable this to show which action ID the robot last executed
	return bRtn;
}

// set the display char for the robot in the world
void InstinctRobot::setRobotChar(const char cChar)
{
	_pWorld->setRobotChar(this, cChar);
}

// get the display char for the robot in the world
char InstinctRobot::getRobotChar(void)
{
	return _pWorld->getRobotChar(this);
}

// get the number of matings since we started
unsigned long InstinctRobot::getMatings(void)
{
	return _ulMatings;
}

// reset the number of matings to zero
void InstinctRobot::resetMatings(void)
{
	_ulMatings = 0;
}

// return a pointer to the plan stored in the robot 
Instinct::CmdPlanner * InstinctRobot::getPlan(void)
{
	return _pPlan;
}

// return a pointer to the names buffer
Instinct::Names * InstinctRobot::getNames(void)
{
	return _pNames;
}

// return a pointer to the monitor plan world stored in the robot 
MonitorPlanWorld * InstinctRobot::getMonitorPlanWorld(void)
{
	return _pMonitorPlanWorld;
}

// The robot needs to sense the world
int InstinctRobot::readSense(const Instinct::senseID nSense)
{
	return _pWorld->readSense(this, nSense);
}

// The robot needs to execute actions in the world 
unsigned char InstinctRobot::executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete)
{
	unsigned char bRtn = _pWorld->executeAction(this, nAction, nActionValue, bCheckForComplete);

	// remember we mated - store time interval for last REMEMBERED_MATE_INTERVALS events
	if ((nAction == ACTION_MATE) && (INSTINCT_RTN(bRtn) == INSTINCT_SUCCESS))
	{
		_ulMatings++;
		_uiMateIntervals[_nIntervalQueue] = _uiMateInterval;
		_uiMateInterval = 0;
		_nIntervalQueue++;
		if (_nIntervalQueue >= REMEMBERED_MATE_INTERVALS)
			_nIntervalQueue = 0;
	}
	else
	{
		_uiMateInterval++;
	}
	return bRtn;
}

// returns the average in the array of mating intervals
// averages the recorded values only
// if no recorded vales return a default
unsigned int InstinctRobot::matingAverage(void)
{
	unsigned long ulAvg = 0;
	unsigned int uiEntries = 0;

	for (unsigned int i = 0; i < REMEMBERED_MATE_INTERVALS; i++)
	{
		if (_uiMateIntervals[i] != INITIAL_MATE_INTERVAL)
		{
			ulAvg += _uiMateIntervals[i];
			uiEntries++;
		}
	}

	if (uiEntries)
		ulAvg = ulAvg / uiEntries;
	else
		ulAvg = INITIAL_MATE_INTERVAL;

	return (unsigned int)ulAvg;
}

// Callback to Monitor plan node executions 
unsigned char InstinctRobot::nodeExecuted(const Instinct::PlanNode * pPlanNode)
{
	return writeLogFile(pPlanNode, "E", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node success 
unsigned char InstinctRobot::nodeSuccess(const Instinct::PlanNode * pPlanNode)
{
	return writeLogFile(pPlanNode, "S", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node pending 
unsigned char InstinctRobot::nodeInProgress(const Instinct::PlanNode * pPlanNode)
{
	return writeLogFile(pPlanNode, "P", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node failure 
unsigned char InstinctRobot::nodeFail(const Instinct::PlanNode * pPlanNode)
{
	return writeLogFile(pPlanNode, "F", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node error 
unsigned char InstinctRobot::nodeError(const Instinct::PlanNode * pPlanNode)
{
	return writeLogFile(pPlanNode, "Z", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan sense measurement 
unsigned char InstinctRobot::nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
	return writeLogFile(NULL, "R", pReleaser, nSenseValue, _szLogFileName, _pNames);
}

// ****** This class provides a world for the MonitorPlan
MonitorPlanWorld::MonitorPlanWorld(InstinctRobot *pInstinctRobot, Instinct::CmdPlanner *pPlan, const char *pMonLogFileName, const unsigned int uiNamesBufferSize)
{
	_pRobot = pInstinctRobot;
	_pParentPlan = pPlan;
	_szMonLogFileName[0] = 0;
	_storedMateDrivePriority = 0;
	_storedRobotChar = '*';


	if (pMonLogFileName)
	{
		memset(_szMonLogFileName, 0, sizeof(_szMonLogFileName));
		strncpy_s(_szMonLogFileName, pMonLogFileName, sizeof(_szMonLogFileName) - 1);
	}

	// create a plan with zero size to start with
	Instinct::instinctID nPlanSize[INSTINCT_NODE_TYPES] = { 0, 0, 0, 0, 0, 0 };
	// note: the senses and actions are handled by callbacks in the MonitorPlanWorld
	_pPlan = new Instinct::CmdPlanner(nPlanSize, this, this, this);
	_pNames = new Instinct::Names(uiNamesBufferSize);
}

// when we destroy the Monitor, also destroy its plan
MonitorPlanWorld::~MonitorPlanWorld()
{
	delete _pPlan;
	delete _pNames;
}

// run the planner. This is its source of CPU resource 
unsigned char MonitorPlanWorld::runPlan(void)
{
	unsigned char bRtn;
	_pPlan->processTimers(1);
	bRtn = _pPlan->runPlan();
	return bRtn;
}

// return a pointer to the plan  
Instinct::CmdPlanner * MonitorPlanWorld::getPlan(void)
{
	return _pPlan;
}

// return a pointer to the names buffer
Instinct::Names * MonitorPlanWorld::getNames(void)
{
	return _pNames;
}

// The monitor needs to sense its world
int MonitorPlanWorld::readSense(const Instinct::senseID nSense)
{
	int nRtn = 0;

	switch (nSense)
	{
	case SENSE_AVG_MATING_INTERVAL:
		nRtn = _pRobot->matingAverage();
		break;
	}

	return nRtn;
}

// The monitor needs to execute actions 
unsigned char MonitorPlanWorld::executeAction(const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete)
{
	unsigned char bSuccess = INSTINCT_FAIL;

	switch (nAction)
	{
	case ACTION_MONITOR_SLEEP:
		bSuccess = sleep(nActionValue, bCheckForComplete);
		break;
	case ACTION_SET_MATE_PRIORITY:
		{
			Instinct::instinctID bMateDriveID = _pRobot->getNames()->getElementID("Mate");
			_storedMateDrivePriority = _pParentPlan->getDrivePriority(bMateDriveID);
			_storedRobotChar = _pRobot->getRobotChar();
			_pRobot->setRobotChar('!');
			if (_pParentPlan->setDrivePriority(bMateDriveID, _storedMateDrivePriority / 2))
				bSuccess = INSTINCT_SUCCESS;
	}
		break;
	case ACTION_RESET_MATE_PRIORITY:
		{
			Instinct::instinctID bMateDriveID = _pRobot->getNames()->getElementID("Mate");
			if (_pParentPlan->setDrivePriority(bMateDriveID, _storedMateDrivePriority))
			{
				_pRobot->setRobotChar(_storedRobotChar);
				_pRobot->initMateIntervals();
				bSuccess = INSTINCT_SUCCESS;
			}
		}
		break;

	default:
		bSuccess = INSTINCT_FAIL;
		break;
	}

	return INSTINCT_RTN_COMBINE(bSuccess, nAction);
}

unsigned char MonitorPlanWorld::sleep(const int nSleepCount, const unsigned char bCheckForComplete)
{
	unsigned char bSuccess;

	if (!bCheckForComplete)
	{
		_sleepRemaining = (nSleepCount > 0) ? (nSleepCount - 1) : 0;
		bSuccess = _sleepRemaining ? INSTINCT_IN_PROGRESS : INSTINCT_SUCCESS;
	}
	else if (!_sleepRemaining)
		bSuccess = INSTINCT_SUCCESS;
	else
	{
		_sleepRemaining--;
		bSuccess = INSTINCT_IN_PROGRESS;
	}

	return INSTINCT_RTN_COMBINE(bSuccess, ACTION_SLEEP);
}


// Callback to Monitor the Monitor plan node executions 
unsigned char MonitorPlanWorld::nodeExecuted(const Instinct::PlanNode * pPlanNode)
{
	return _pRobot->writeLogFile(pPlanNode, "E", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node success 
unsigned char MonitorPlanWorld::nodeSuccess(const Instinct::PlanNode * pPlanNode)
{
	return _pRobot->writeLogFile(pPlanNode, "S", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node pending 
unsigned char MonitorPlanWorld::nodeInProgress(const Instinct::PlanNode * pPlanNode)
{
	return _pRobot->writeLogFile(pPlanNode, "P", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node failure 
unsigned char MonitorPlanWorld::nodeFail(const Instinct::PlanNode * pPlanNode)
{
	return _pRobot->writeLogFile(pPlanNode, "F", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node error 
unsigned char MonitorPlanWorld::nodeError(const Instinct::PlanNode * pPlanNode)
{
	return _pRobot->writeLogFile(pPlanNode, "Z", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan sense measurement 
unsigned char MonitorPlanWorld::nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
	return _pRobot->writeLogFile(NULL, "R", pReleaser, nSenseValue, _szMonLogFileName, _pNames);
}



unsigned char InstinctRobot::writeLogFile(const Instinct::PlanNode * pPlanNode, const char *pType, const Instinct::ReleaserType *pReleaser,
											const int nSenseValue, const char *pLogFileName, Instinct::Names *pNames)
{
	if (strlen(_szLogFileName) > 4)
	{
		struct tm newtime;
		__time64_t long_time;
		char szTimeBuff[26];
		errno_t err;
		FILETIME sFileTime;
		std::ofstream logfile;
		char szSystemTimeBuff[30];

		// Get time as 64-bit integer.
		_time64(&long_time);
		// Convert to local time.
		err = _localtime64_s(&newtime, &long_time);
		if (err)
			return false;
		GetSystemTimeAsFileTime(&sFileTime);
		sprintf_s(szSystemTimeBuff, "%010u.%010u", sFileTime.dwHighDateTime, sFileTime.dwLowDateTime);
		// Convert time to an ASCII representation.
		sprintf_s(szTimeBuff, "%04u-%02u-%02u %02u:%02u:%02u", 1900 + newtime.tm_year, 1 + newtime.tm_mon, newtime.tm_mday,
																newtime.tm_hour, newtime.tm_min, newtime.tm_sec);
		// err = asctime_s(szTimeBuff, 26, &newtime);
		if (err)
			return false;

		logfile.open(pLogFileName, std::fstream::in | std::fstream::out | std::fstream::app);
		if (logfile.is_open())
		{
			char szDisplayBuff[80];
			logfile << szTimeBuff << " " << szSystemTimeBuff << " " << pType << " ";
			if (pPlanNode)
			{
				_pPlan->displayNodeCounters(szDisplayBuff, sizeof(szDisplayBuff), pPlanNode);
				Instinct::instinctID nID = pPlanNode->sElement.sReferences.bRuntime_ElementID;
				char *pName = pNames->getElementName(nID);
				if (pName)
				{
					logfile << pNodeTypeNames[pPlanNode->bNodeType] << " " << pName << " " << szDisplayBuff << "\n";
				}
				else
				{
					logfile << pNodeTypeNames[pPlanNode->bNodeType] << " " << pPlanNode->sElement.sReferences.bRuntime_ElementID << " " << szDisplayBuff << "\n";
				}
			}
			if (pReleaser)
			{
				_pPlan->displayReleaser(szDisplayBuff, sizeof(szDisplayBuff), pReleaser);
				logfile << szDisplayBuff << " " << nSenseValue << "\n";
			}
			logfile.close();
		}
	}
	return true;
}

