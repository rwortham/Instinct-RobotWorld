// A Robot that can operate in InstinctWorld, using the Instinct Planner
#include "stdafx.h"
#include "Instinct.h"
#include "InstinctRobot.h"
#include "InstinctWorld.h"


// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


const char *pNodeTypeNames[INSTINCT_NODE_TYPES] =
{	{ "AP" },
	{ "APE" },
	{ "C" },
	{ "CE" },
	{ "D" },
	{ "A" },
};

unsigned char bWinsockInitialised = false;

InstinctRobot::InstinctRobot(InstinctWorld *pWorld, const char cRobot, const char *pLogFileName, const char *pMonLogFileName, const unsigned int uiNamesBufferSize)
{
	_szLogFileName[0] = 0;
	_uiMateInterval = 0;
	_nIntervalQueue = 0;
	_ulMatings = 0;
	_szHostName[0] = 0;
	_szPort[0] = 0;
	_connectSocket = INVALID_SOCKET;
	
	GetSystemTimeAsFileTime(&sStartTime); // store the time when the robot instance is created

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

// cleanup
InstinctRobot::~InstinctRobot()
{
	char recvbuf[1024];
	int iResult;

	// close socket if it is open
	if (_connectSocket != INVALID_SOCKET)
	{
		// MessageBox(NULL, TEXT("Closing socket"), NULL, MB_OK);
		// attempt a graceful close with handshake from the server
		iResult = shutdown(_connectSocket, SD_SEND);
		if (iResult != SOCKET_ERROR)
		{
			// MessageBox(NULL, TEXT("Graceful socket close"), NULL, MB_OK);
			// read all the data from the socket until the remote end disconnects nicely
			while ((iResult = recv(_connectSocket, recvbuf, sizeof(recvbuf), 0)) > 0);
		}
		closesocket(_connectSocket);
		_connectSocket = INVALID_SOCKET;
	}


	// when we destroy the robot, also destroy its plan
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


// store the Instinct Server parameters for remote logging
void InstinctRobot::setInstinctServerParams(char *pHostName, char *pPort, char *pHostName2, char *pPort2)
{
	strncpy(_szHostName, pHostName, sizeof(_szHostName));
	strncpy(_szPort, pPort, sizeof(_szPort));
	_pMonitorPlanWorld->setHostParams(pHostName2, pPort2);
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
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		writeInstinctServer(pPlanNode, "E", NULL, 0, FALSE);
	return writeLogFile(pPlanNode, "E", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node success 
unsigned char InstinctRobot::nodeSuccess(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		writeInstinctServer(pPlanNode, "S", NULL, 0, FALSE);
	return writeLogFile(pPlanNode, "S", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node pending 
unsigned char InstinctRobot::nodeInProgress(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		writeInstinctServer(pPlanNode, "P", NULL, 0, FALSE);
	return writeLogFile(pPlanNode, "P", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node failure 
unsigned char InstinctRobot::nodeFail(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		writeInstinctServer(pPlanNode, "F", NULL, 0, FALSE);
	return writeLogFile(pPlanNode, "F", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan node error 
unsigned char InstinctRobot::nodeError(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		writeInstinctServer(pPlanNode, "Z", NULL, 0, FALSE);
	return writeLogFile(pPlanNode, "Z", NULL, 0, _szLogFileName, _pNames);
}

// Callback to Monitor plan sense measurement 
unsigned char InstinctRobot::nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		writeInstinctServer(NULL, "R", pReleaser, nSenseValue, FALSE);
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
	_szHostName[0] = 0;
	_szPort[0] = 0;
	_connectSocket = INVALID_SOCKET;


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
	char recvbuf[1024];
	int iResult;

	// close socket if it is open
	if (_connectSocket != INVALID_SOCKET)
	{
		// MessageBox(NULL, TEXT("Closing Monitor socket"), NULL, MB_OK);
		// attempt a graceful close with handshake from the server
		iResult = shutdown(_connectSocket, SD_SEND);
		if (iResult != SOCKET_ERROR)
		{
			// MessageBox(NULL, TEXT("Graceful monitor socket close"), NULL, MB_OK);
			// read all the data from the socket until the remote end disconnects nicely
			while ((iResult = recv(_connectSocket, recvbuf, sizeof(recvbuf), 0)) > 0);
		}
		closesocket(_connectSocket);
		_connectSocket = INVALID_SOCKET;
	}

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

SOCKET MonitorPlanWorld::getConnectSocket(void)
{
	return _connectSocket;
}

void MonitorPlanWorld::setConnectSocket(SOCKET socket)
{
	_connectSocket = socket;
}

void MonitorPlanWorld::setHostParams(char *pHostName, char *pPort)
{
	strncpy(_szHostName, pHostName, sizeof(_szHostName));
	strncpy(_szPort, pPort, sizeof(_szPort));
}

char * MonitorPlanWorld::getHostName(void)
{
	return _szHostName;
}

char * MonitorPlanWorld::getPort(void)
{
	return _szPort;
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
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		_pRobot->writeInstinctServer(pPlanNode, "E", NULL, 0, TRUE);
	return _pRobot->writeLogFile(pPlanNode, "E", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node success 
unsigned char MonitorPlanWorld::nodeSuccess(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		_pRobot->writeInstinctServer(pPlanNode, "S", NULL, 0, TRUE);
	return _pRobot->writeLogFile(pPlanNode, "S", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node pending 
unsigned char MonitorPlanWorld::nodeInProgress(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		_pRobot->writeInstinctServer(pPlanNode, "P", NULL, 0, TRUE);
	return _pRobot->writeLogFile(pPlanNode, "P", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node failure 
unsigned char MonitorPlanWorld::nodeFail(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		_pRobot->writeInstinctServer(pPlanNode, "F", NULL, 0, TRUE);
	return _pRobot->writeLogFile(pPlanNode, "F", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan node error 
unsigned char MonitorPlanWorld::nodeError(const Instinct::PlanNode * pPlanNode)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		_pRobot->writeInstinctServer(pPlanNode, "Z", NULL, 0, TRUE);
	return _pRobot->writeLogFile(pPlanNode, "Z", NULL, 0, _szMonLogFileName, _pNames);
}

// Callback to Monitor the Monitor plan sense measurement 
unsigned char MonitorPlanWorld::nodeSense(const Instinct::ReleaserType *pReleaser, const int nSenseValue)
{
	if ((strlen(_szPort) > 0) && (strlen(_szHostName) > 0))
		_pRobot->writeInstinctServer(NULL, "R", pReleaser, nSenseValue, TRUE);
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
		std::ofstream logfile;
		char szSystemTimeBuff[20];
		FILETIME sFileTime;
		_ULARGE_INTEGER uLargeNow;
		_ULARGE_INTEGER uLargeStart;
		unsigned long ulElapsedTime;

		// Get time as 64-bit integer.
		_time64(&long_time);
		// Convert to local time.
		err = _localtime64_s(&newtime, &long_time);
		if (err)
			return false;
		GetSystemTimeAsFileTime(&sFileTime);

		// calculate elapsed time since the robot was initialised - to match output with R5
		GetSystemTimeAsFileTime(&sFileTime); // gets time in 100nS intervals. We want mS so / 10,000
		uLargeNow.u.LowPart = sFileTime.dwLowDateTime;
		uLargeNow.u.HighPart = sFileTime.dwHighDateTime;
		uLargeStart.u.LowPart = sStartTime.dwLowDateTime;
		uLargeStart.u.HighPart = sStartTime.dwHighDateTime;
		ulElapsedTime = (uLargeNow.QuadPart - uLargeStart.QuadPart) / 10000L;

		sprintf_s(szSystemTimeBuff, "%010lu", ulElapsedTime);

		// Convert time to an ASCII representation.
		sprintf_s(szTimeBuff, "%04u-%02u-%02u %02u:%02u:%02u", 1900 + newtime.tm_year, 1 + newtime.tm_mon, newtime.tm_mday,
																newtime.tm_hour, newtime.tm_min, newtime.tm_sec);

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

// write output to Instinct Server. bMonitorPlan defines which socket to use.
// Create and open the Socket on first invocation. If error occurs then don't try again.
unsigned char InstinctRobot::writeInstinctServer(const Instinct::PlanNode * pPlanNode, const char *pType, const Instinct::ReleaserType *pReleaser, const int nSenseValue, const unsigned char bMonitorPlan)
{
	WSADATA wsaData;
	int iResult;
	SOCKET mySocket;
	struct addrinfo *result = NULL, *ptr = NULL, hints;
	char szBuffer[200];
	wchar_t szErrorMsg[100];

	// Initialize Winsock
	if (!bWinsockInitialised)
	{
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (iResult != 0)
		{
			MessageBox(NULL, TEXT("WSAStartup failed."), TEXT("Windows Socket Error"), MB_ICONWARNING | MB_OK);
			return false;
		}
		bWinsockInitialised = true;
	}

	// check if the correct socket is already open, if not attempt to open it
	if (bMonitorPlan)
		mySocket = _pMonitorPlanWorld->getConnectSocket();
	else
		mySocket = _connectSocket;

	if (mySocket == INVALID_SOCKET)
	{
		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_UNSPEC;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = IPPROTO_TCP;

		// Resolve the server address and port
		iResult = getaddrinfo((bMonitorPlan ? _pMonitorPlanWorld->getHostName() : _szHostName), (bMonitorPlan ? _pMonitorPlanWorld->getPort() : _szPort), &hints, &result);
		if (iResult != 0)
		{
			wcscpy(szErrorMsg, TEXT("getaddrinfo failed - Hostname not found"));
		}
		else
		{
			// Attempt to connect to an address until one succeeds
			for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
			{

				// Create a SOCKET for connecting to server
				mySocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
				if (mySocket == INVALID_SOCKET)
				{
					wcscpy(szErrorMsg, TEXT("socket failed - Create Socket"));
				}
				else
				{
					// Connect to server.
					iResult = connect(mySocket, ptr->ai_addr, (int)ptr->ai_addrlen);
					if (iResult == SOCKET_ERROR)
					{
						closesocket(mySocket);
						mySocket = INVALID_SOCKET;
						continue;
					}
					break;
				}
			}
			freeaddrinfo(result);

			if (mySocket == INVALID_SOCKET)
			{
				wcscpy(szErrorMsg, TEXT("Unable to connect to server - Connect Socket"));
			}
			else
			{
				strcpy(szBuffer, "Robot Running ...\n");
				iResult = send(mySocket, szBuffer, (int)strlen(szBuffer), 0);
				if (iResult == SOCKET_ERROR)
				{
					wcscpy(szErrorMsg, TEXT("Socket send failed - Robot Running Message"));
					closesocket(mySocket);
					mySocket = INVALID_SOCKET;
				}
			}
		}
	}

	// write log data to the correct socket
	if (mySocket != INVALID_SOCKET)
	{
		FILETIME sFileTime;
		_ULARGE_INTEGER uLargeNow;
		_ULARGE_INTEGER uLargeStart;
		unsigned long ulElapsedTime;

		// calculate elapsed time since the robot was initialised - to match output with R5
		GetSystemTimeAsFileTime(&sFileTime); // gets time in 100nS intervals. We want mS so / 10,000
		uLargeNow.u.LowPart = sFileTime.dwLowDateTime;
		uLargeNow.u.HighPart = sFileTime.dwHighDateTime;
		uLargeStart.u.LowPart = sStartTime.dwLowDateTime;
		uLargeStart.u.HighPart = sStartTime.dwHighDateTime;
		ulElapsedTime = (uLargeNow.QuadPart - uLargeStart.QuadPart) / 10000L;

		char szDisplayBuff[80];
		Instinct::CmdPlanner *pPlan = bMonitorPlan ? _pMonitorPlanWorld->getPlan() : _pPlan;
		if (pPlanNode)
		{
			pPlan->displayNodeCounters(szDisplayBuff, sizeof(szDisplayBuff), pPlanNode);
			sprintf_s(szBuffer, "%010lu %s %s %s\n", ulElapsedTime, pType, pNodeTypeNames[pPlanNode->bNodeType], szDisplayBuff);
		}
		if (pReleaser)
		{
			pPlan->displayReleaser(szDisplayBuff, sizeof(szDisplayBuff), pReleaser);
			sprintf_s(szBuffer, "%010lu %s %s %i\n", ulElapsedTime, pType, szDisplayBuff, nSenseValue);
		}

		iResult = send(mySocket, szBuffer, (int)strlen(szBuffer), 0);
		if (iResult == SOCKET_ERROR)
		{
			wcscpy(szErrorMsg, TEXT("Socket send failed - Socket"));
			closesocket(mySocket);
			mySocket = INVALID_SOCKET;
		}
	}

	if (bMonitorPlan)
		_pMonitorPlanWorld->setConnectSocket(mySocket);
	else
		_connectSocket = mySocket;

	if (mySocket == INVALID_SOCKET)
	{
		if (bMonitorPlan)
		{
			_pMonitorPlanWorld->setHostParams("", ""); // stop attempt to reconnect socket next time round
		}
		else
		{
			_szHostName[0] = 0;
			_szPort[0] = 0;
		}

		MessageBox(NULL, szErrorMsg, TEXT("Comms Error"), MB_OK | MB_ICONQUESTION);
	}

	return (mySocket == INVALID_SOCKET ? false : true);
}

