// InstinctWorld provides a very simple test harness to enable testing of the Instinct Planner.
// Its purpose was originally to test and demonstrate the planner, but it provides a platform on which more
// complex worlds can be built. As such, it is a minimal and simple implementation, though well structured.

#include "stdafx.h"
#include "Instinct.h"
#include "InstinctRobot.h"
#include "InstinctWorld.h"

InstinctWorld::InstinctWorld()
{
	initialiseWorld();
}


// the world is ending, so empty my list of robots
InstinctWorld::~InstinctWorld()
{
	std::list <InstinctWorldRobot>::iterator robotIter;
	// TODO
}

// clock the world
void InstinctWorld::tick(void)
{
	unsigned char bChar;
	// calc new positions for robots
	for (int i = _robots.size(); i > 0; i--)
	{
		InstinctWorldRobot aRobot = _robots.front();
		aRobot.move(&bChar);
		_robots.pop_front();
		_robots.push_back(aRobot);
	}
}

// add the robot into the world, by adding it and its position to my list of robots
void InstinctWorld::addRobot(InstinctRobot *pRobot, const unsigned int nXpos, const unsigned int nYpos, const int nHeading, const char cRobot)
{
	InstinctWorldRobot mRobot(this, pRobot, nXpos, nYpos, nHeading, cRobot);
	_robots.push_back(mRobot);
}

// remove a robot from the world
void InstinctWorld::removeRobot(InstinctRobot *pRobot)
{
	std::list <InstinctWorldRobot>::iterator robotIter;
	const InstinctWorldRobot search(this, pRobot, 0, 0, 0, ' ');

	// find the correct robot, and get its position
	robotIter = std::find(_robots.begin(), _robots.end(), search);
	if (robotIter != _robots.end())
	{
		_robots.erase(robotIter);
	}
}

// this is the entry point into the world where sensors are read
int InstinctWorld::readSense(InstinctRobot *pRobot, const Instinct::senseID nSense)
{
	int nRtn = 0;

	switch (nSense)
	{
		case SENSE_FRONT:
		case SENSE_REAR:
		case SENSE_FRONT_RIGHT:
		case SENSE_FRONT_LEFT:
		case SENSE_REAR_RIGHT:
		case SENSE_REAR_LEFT:
			nRtn = readCornerSensors(pRobot, nSense);
			break;
		case SENSE_RANDOM:
			nRtn = senseRandom(pRobot);
			break;
		case SENSE_FIFTY: // always return 50
			nRtn = 50;
			break;
		case SENSE_MATE:
			nRtn = senseMate(pRobot);
			break;
	}

	return nRtn;
}
// this private method moves a phantom robot around to test whether it would bump into things, and
// thus what the sensors should return
int InstinctWorld::readCornerSensors(InstinctRobot *pRobot, const Instinct::senseID nSense)
{
	std::list <InstinctWorldRobot>::iterator robotIter;
	const InstinctWorldRobot search(this, pRobot, 0, 0, 0, ' ');
	int nDist = 0;
	unsigned char bSuccess = INSTINCT_SUCCESS;
	unsigned char bChar;

	// find the correct robot, and get its position
	robotIter = std::find(_robots.begin(), _robots.end(), search);
	InstinctWorldRobot theRobot = *robotIter;

	// remember theRobot is in fact a copy of the robot, so is a phantom. We can find the distance to obstacles
	// by moving the phantom in the correct direction and then testing for bumping in to things
	switch (nSense)
	{
	case SENSE_FRONT: // move the phantom robot forward
		theRobot.setSpeed(1, false); // forward one unit per tick
		break;
	case SENSE_REAR:
		theRobot.setSpeed(-1, false); // back one unit per tick
		break;
	case SENSE_FRONT_RIGHT:
		theRobot.setSpeed(1, false);
		theRobot.turn(45, false);
		break;
	case SENSE_FRONT_LEFT:
		theRobot.setSpeed(1, false);
		theRobot.turn(-45, false);
		break;
	case SENSE_REAR_RIGHT:
		theRobot.setSpeed(-1, false);
		theRobot.turn(45, false);
		break;
	case SENSE_REAR_LEFT:
		theRobot.setSpeed(-1, false);
		theRobot.turn(-45, false);
		break;
	}
	while ((bSuccess == INSTINCT_SUCCESS) && (nDist < 4))
	{
		bSuccess = theRobot.move(&bChar);
		if (bSuccess == INSTINCT_SUCCESS)
			nDist++;
		else
			break; // if we are stuck don't waste time
	}
	// now read the sense value
	return nDist;
}

// just returns a random number 1-100
int InstinctWorld::senseRandom(InstinctRobot *pRobot)
{
	int nRand;

	nRand = rand();
	nRand = min(100, 1 + (((long)nRand*100L)/RAND_MAX));
	
	return nRand;
}

// return true if there is a mate
int InstinctWorld::senseMate(InstinctRobot *pRobot)
{
	std::list <InstinctWorldRobot>::iterator robotIter;
	const InstinctWorldRobot search(this, pRobot, 0, 0, 0, ' ');

	// find the correct robot
	robotIter = std::find(_robots.begin(), _robots.end(), search);
	InstinctWorldRobot theRobot = *robotIter;

	return otherExists(&theRobot) ? 1 : 0;
}


// execute the action on the correct robot
// valid actions are ACTION_ # defines
unsigned char InstinctWorld::executeAction(InstinctRobot *pRobot, const Instinct::actionID nAction, const int nActionValue, const unsigned char bCheckForComplete)
{
	unsigned char bSuccess;
	std::list <InstinctWorldRobot>::iterator robotIter;

	const InstinctWorldRobot search(this, pRobot, 0, 0, 0, ' ');
	// find the correct robot, and get its position
	robotIter = std::find(_robots.begin(), _robots.end(), search);
	InstinctWorldRobot theRobot = *robotIter;

	switch (nAction)
	{
	case ACTION_SETSPEED:
		bSuccess = theRobot.setSpeed(nActionValue, bCheckForComplete);
		break;
	case ACTION_MOVEBY:
		bSuccess = theRobot.moveBy(nActionValue, bCheckForComplete);
		break;
	case ACTION_TURN:
		bSuccess = theRobot.turn(nActionValue, bCheckForComplete);
		break;
	case ACTION_STOP:
		bSuccess = theRobot.stop(nActionValue, bCheckForComplete);
		break;
	case ACTION_SLEEP:
		bSuccess = theRobot.sleep(nActionValue, bCheckForComplete);
		break;
	case ACTION_SUCCEED: // always succeeds - useful for testing
		bSuccess = INSTINCT_SUCCESS;
		break;
	case ACTION_MATE:
		bSuccess = theRobot.mate(nActionValue, bCheckForComplete);
		break;
	case ACTION_RANDOM_TURN:
		bSuccess = theRobot.randomTurn(nActionValue, bCheckForComplete);
		break;

	case ACTION_FAIL: // always fails - useful for testing
	default:
		bSuccess = INSTINCT_FAIL;
		break;
	}
	_robots.erase(robotIter);
	_robots.push_front(theRobot); // self modifing list - most freqently updated bots are nearest front of list

	return bSuccess;
}

// Allows the robot to get its display char in the world
char InstinctWorld::getRobotChar(InstinctRobot *pRobot)
{
	std::list <InstinctWorldRobot>::iterator robotIter;

	const InstinctWorldRobot search(this, pRobot, 0, 0, 0, ' ');
	// find the correct robot
	robotIter = std::find(_robots.begin(), _robots.end(), search);
	InstinctWorldRobot theRobot = *robotIter;

	return theRobot.getRobotChar();
}

// Allows the robot to set its display char in the world
void InstinctWorld::setRobotChar(InstinctRobot *pRobot, const char cChar)
{
	std::list <InstinctWorldRobot>::iterator robotIter;

	const InstinctWorldRobot search(this, pRobot, 0, 0, 0, ' ');
	// find the correct robot
	robotIter = std::find(_robots.begin(), _robots.end(), search);
	InstinctWorldRobot theRobot = *robotIter;

	theRobot.setRobotChar(cChar);
	_robots.erase(robotIter);
	_robots.push_front(theRobot);
}

// for now, this is hard coded. Could read from a file
void InstinctWorld::initialiseWorld(void)
{
	// Seed the random-number generator with current time so that
	// the numbers will be different every time we run.
	srand((unsigned)time(NULL));

	memset(cMap, ' ', sizeof(cMap));
	// walls around the map
	for (int i = 0; i < WORLD_X; i++)
	{
		cMap[0][i] = 'X';
		cMap[WORLD_Y - 1][i] = 'X';
	}
	for (int i = 0; i < WORLD_Y; i++)
	{
		cMap[i][0] = 'X';
		cMap[i][WORLD_X - 1] = 'X';
	}

	// obstacles
	// diagonal bar
	for (int i = 1; i < 15; i++)
	{
		cMap[i][i] = cMap[i][i+1] = 'X';
	}

	// vertical bar
	for (int i = 0; i < WORLD_Y / 2; i++)
	{
		cMap[WORLD_Y / 4 + i ][WORLD_X / 4] = 'X';
	}
	// horizontal bar
	for (int i = 0; i < WORLD_X / 3; i++)
	{
		cMap[WORLD_Y/3][WORLD_X/3 + i] = 'X';
	}
	// robot trap
	for (int i = WORLD_Y / 2; i < WORLD_Y; i++)
	{
		cMap[i][WORLD_X - 9] = 'X';
	}

}

// this is really basic, it reads the map into a buffer, overlays the robots and prints to screen
void InstinctWorld::showWorld(void)
{
	unsigned char cTempMap[WORLD_Y][WORLD_X+1]; // add extra character to achieve zero terminated strings

	memset(cTempMap, 0, sizeof(cTempMap)); // initialise display map to nulls
	for (int i = 0; i < WORLD_Y; i++) // copy the actual map
		memcpy(cTempMap[i], cMap[i], WORLD_X);

	// now overlay the positions of the robots
	for (std::list <InstinctWorldRobot>::iterator robotIter = _robots.begin(); robotIter != _robots.end(); ++robotIter)
	{
		InstinctWorldRobot aRobot = *robotIter;
		cTempMap[aRobot.nYpos][aRobot.nXpos] = aRobot.robotChar;
	}

	// print one line at a time
	for (int i = 0; i < WORLD_Y; i++)
	{
		printf("%s\n", cTempMap[i]);
	}
}

// this is really basic, it reads the map into a buffer, overlays the robots and prints to a string buffer
void InstinctWorld::showWorld(wchar_t *pBuffer, const unsigned int nBuffLen)
{
	unsigned char cTempMap[WORLD_Y][WORLD_X + 1]; // add extra character to achieve zero terminated strings

	memset(cTempMap, 0, sizeof(cTempMap)); // initialise display map to nulls
	for (int i = 0; i < WORLD_Y; i++) // copy the actual map
		memcpy(cTempMap[i], cMap[i], WORLD_X);

	// now overlay the positions of the robots
	for (std::list <InstinctWorldRobot>::iterator robotIter = _robots.begin(); robotIter != _robots.end(); ++robotIter)
	{
		InstinctWorldRobot aRobot = *robotIter;
		cTempMap[aRobot.nYpos][aRobot.nXpos] = aRobot.robotChar;
	}

	// print one line at a time to the output buffer
	*pBuffer = L'\0'; // zero terminate the buffer to start
	for (int i = 0; i < WORLD_Y; i++)
	{
		int nLen = wcslen(pBuffer);
		_snwprintf(pBuffer + nLen, nBuffLen - nLen, L"%S\r\n", cTempMap[i]);
	}
}

// return number of moving robots
unsigned int InstinctWorld::movingRobots(void)
{
	int nMoving = 0;

	for (std::list <InstinctWorldRobot>::iterator robotIter = _robots.begin(); robotIter != _robots.end(); ++robotIter)
	{
		InstinctWorldRobot aRobot = *robotIter;
		if (aRobot.getSpeed()) // just count those with non zero speeds
			nMoving++;
	}

	return nMoving;
}

// return total number of robots
unsigned int InstinctWorld::totalRobots(void)
{
	return _robots.size();
}


// return true if there is another robot in the world in the same position as pIWRobot
InstinctRobot * InstinctWorld::otherExists(InstinctWorldRobot *pIWRobot)
{
	for (std::list <InstinctWorldRobot>::iterator robotIter = _robots.begin(); robotIter != _robots.end(); ++robotIter)
	{
		InstinctWorldRobot aRobot = *robotIter;

		if (aRobot.pRobot != pIWRobot->pRobot) // don't find yourself
		{
			if ((aRobot.nXpos == pIWRobot->nXpos) && (aRobot.nYpos == pIWRobot->nYpos))
				return aRobot.pRobot;
		}
	}
	return NULL;
}

InstinctWorldRobot::InstinctWorldRobot(InstinctWorld *pWorld, InstinctRobot *pMyRobot, const unsigned int nX, const unsigned int nY, const int nHeading, const char cRobot)
{
	pRobot = pMyRobot;
	_pWorld = pWorld;
	_nHeading = nHeading;
	_newHeading = 0;
	_moveRemaining = 0;
	_sleepRemaining = 0;
	_nSpeed = 0;
	nXpos = nX;
	nYpos = nY;
	robotChar = cRobot;
	_turnInProgress = false;
	_moveInProgress = false;
}

InstinctWorldRobot::~InstinctWorldRobot()
{
}

// comparator needed for std::find
bool InstinctWorldRobot::operator == (InstinctWorldRobot const & rhs)
{
		return (pRobot == rhs.pRobot);
}

// assignment needed for *iter
void InstinctWorldRobot::operator = (InstinctWorldRobot const & rhs)
{
	_pWorld = rhs._pWorld;
	pRobot = rhs.pRobot;
	_nHeading = rhs._nHeading;
	_newHeading = rhs._newHeading;
	_nSpeed = rhs._nSpeed;
	_moveRemaining = rhs._moveRemaining;
	_sleepRemaining = rhs._sleepRemaining;
	_turnInProgress = rhs._turnInProgress;
	_moveInProgress = rhs._moveInProgress;
	nXpos = rhs.nXpos;
	nYpos = rhs.nYpos;
	robotChar = rhs.robotChar;
}

// calculates the new robot position based on heading, speed and so on
// returns INSTINCT_SUCCESS unless robot hits something
// if successful then the contents of the new square are filled into *pbChar
unsigned char InstinctWorldRobot::move(unsigned char *pbChar)
{
	unsigned char bSuccess = INSTINCT_SUCCESS;

	//number of units to move depends on nSpeed
	int nDir = (_nSpeed > 0) ? 1 : -1;
	for (int i = _nSpeed; i != 0; (nDir > 0) ? i-- : i++)
	{
		bSuccess = INSTINCT_FAIL;
		// there are only 8 valid headings in our restricted world
		switch (_nHeading)
		{
		case 0: // north
			if ((nYpos > 0) && ((*pbChar = _pWorld->cMap[nYpos - nDir][nXpos]) != 'X'))
			{
				nYpos -= nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 45: // north east
			if ((nYpos > 0) && (nXpos < WORLD_X) && ((*pbChar = _pWorld->cMap[nYpos - nDir][nXpos + nDir]) != 'X'))
			{
				nXpos += nDir;
				nYpos -= nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 90: // east
			if ((nXpos < WORLD_X) && ((*pbChar = _pWorld->cMap[nYpos][nXpos + nDir]) != 'X'))
			{
				nXpos += nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 135: // south east
			if ((nYpos < WORLD_Y) && (nXpos < WORLD_X) && ((*pbChar = _pWorld->cMap[nYpos + nDir][nXpos + nDir]) != 'X'))
			{
				nXpos += nDir;
				nYpos += nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 180: // south
			if ((nYpos < WORLD_Y) && ((*pbChar = _pWorld->cMap[nYpos + nDir][nXpos]) != 'X'))
			{
				nYpos += nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 225: // south west
			if ((nYpos < WORLD_Y) && (nXpos > 0) && ((*pbChar = _pWorld->cMap[nYpos + nDir][nXpos - nDir]) != 'X'))
			{
				nYpos += nDir;
				nXpos -= nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 270: // west
			if ((nXpos > 0) && ((*pbChar = _pWorld->cMap[nYpos][nXpos - nDir]) != 'X'))
			{
				nXpos -= nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		case 315: // north west
			if ((nXpos > 0) && (nYpos > 0) && ((*pbChar = _pWorld->cMap[nYpos - nDir][nXpos - nDir]) != 'X'))
			{
				nXpos -= nDir;
				nYpos -= nDir;
				bSuccess = INSTINCT_SUCCESS;
			}
			break;
		}
	}
	return bSuccess;
}

// define the new heading - only increments of 45' are allowed
// robot only turns 45' in each cycle
unsigned char InstinctWorldRobot::turn(const int nDegrees, const unsigned char bCheckForComplete)
{
	int nTurn;

	if (nDegrees % 45 != 0)
		return INSTINCT_ERROR;

	if (!bCheckForComplete)
		_turnInProgress = false;

	if (!_turnInProgress)
	{
		_newHeading = (_nHeading + nDegrees + 360) % 360;
	}

	nTurn = (nDegrees > 0) ? 45 : -45;
	_nHeading = (_nHeading + nTurn + 360) % 360;

	if (_nHeading == _newHeading)
	{
		_turnInProgress = false;
		return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_TURN);
	}
	else
	{
		_turnInProgress = true;
		return INSTINCT_RTN_COMBINE(INSTINCT_IN_PROGRESS, ACTION_TURN);
	}
}

// randomly choose to turn left or right
unsigned char InstinctWorldRobot::randomTurn(const int nDegrees, const unsigned char bCheckForComplete)
{
	int nTurn;

	if (nDegrees % 45 != 0)
		return INSTINCT_ERROR;

	if (!bCheckForComplete)
		_turnInProgress = false;

	if (!_turnInProgress)
	{
		int nDegs;
		int nRand;
		nRand = rand();
		nDegs = (nRand > RAND_MAX / 2) ? nDegrees : 0-nDegrees;
		_newHeading = (_nHeading + nDegs + 360) % 360;
	}

	nTurn = (nDegrees > 0) ? 45 : -45;
	_nHeading = (_nHeading + nTurn + 360) % 360;

	if (_nHeading == _newHeading)
	{
		_turnInProgress = false;
		return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_TURN);
	}
	else
	{
		_turnInProgress = true;
		return INSTINCT_RTN_COMBINE(INSTINCT_IN_PROGRESS, ACTION_TURN);
	}
}

// speed in units per cycle
unsigned char InstinctWorldRobot::setSpeed(const int nSpeed, const unsigned char bCheckForComplete)
{
	_nSpeed = nSpeed;
	return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_SETSPEED);
}

// allows robot to define how far it moves (since it can use its own odometry)
// this is a big simplification in the robot simulator, but ok for plan logic testing
// because we are not implementing the behaviours, but simply turning plan actions into
// immediate movements in the world
unsigned char InstinctWorldRobot::moveBy(const int nDistance, const unsigned char bCheckForComplete)
{
	int nSpeed = _nSpeed;
	unsigned char bSuccess = INSTINCT_SUCCESS;
	unsigned char bChar;

	if (!bCheckForComplete)
		_moveInProgress = false;

	if (!_moveInProgress)
		_moveRemaining = nDistance;

	_nSpeed = (nDistance > 0) ? 1 : -1;
	if (!nDistance)
		_nSpeed = 0;

	bSuccess = move(&bChar);
	if (bSuccess == INSTINCT_SUCCESS)
	{
		_moveRemaining -= _nSpeed;

		if (_moveRemaining)
		{
			_moveInProgress = true;
			bSuccess = INSTINCT_IN_PROGRESS;
		}
		else
		{
			_moveInProgress = false;
		}
	}
	else
	{
		_moveInProgress = false;
	}
	_nSpeed = 0; // after a move attempt we always stop

	return INSTINCT_RTN_COMBINE(bSuccess, ACTION_MOVEBY);
}

// robot stops and sleeps for nSleepCount. Might be interrupted!
unsigned char InstinctWorldRobot::sleep(const int nSleepCount, const unsigned char bCheckForComplete)
{
	unsigned char bSuccess;
	
	if (!bCheckForComplete)
	{
		_moveInProgress = false;
		_turnInProgress = false;
		_nSpeed = 0; // stop the robot
		_sleepRemaining = (nSleepCount > 0) ? (nSleepCount-1) : 0;
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

// perform a hard stop
unsigned char InstinctWorldRobot::stop(const int nActionValue, const unsigned char bCheckForComplete)
{
	_moveInProgress = false;
	_turnInProgress = false;
	_nSpeed = 0;

	return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_STOP);
}

// ask the world if there is a conspecific in my position.
// return true if there is
unsigned char InstinctWorldRobot::mate(const int nActionValue, const unsigned char bCheckForComplete)
{
	if (_pWorld->otherExists(this))
		return INSTINCT_RTN_COMBINE(INSTINCT_SUCCESS, ACTION_MATE);
	else
		return INSTINCT_RTN_COMBINE(INSTINCT_FAIL, ACTION_MATE);
}

// set the display char for the robot in the world
void InstinctWorldRobot::setRobotChar(const char cChar)
{
	robotChar = cChar;
}

// get the display char for the robot in the world
char InstinctWorldRobot::getRobotChar(void)
{
	return robotChar;
}

// return the robot's speed in the world
int InstinctWorldRobot::getSpeed(void)
{
	return _nSpeed;
}