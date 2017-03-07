// Instinct-RobotWorld.cpp : Defines the entry point for the application.
//  Copyright (c) 2016  Robert H. Wortham <r.h.wortham@gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#include "stdafx.h"
#include "Instinct.h"
#include "InstinctRobot.h"
#include "InstinctWorld.h"
#include "InstinctRobotWorld.h"

#define NAMES_BUFFER_SIZE 2000
#define MAX_ROBOTS 1000 // how many robots can we run

#define MAKE_ROBOTS 6 // how many robots to create
#define DEFAULT_RATE 8 // the default rate at which the world runs
const wchar_t szDefaultPlanFile[] = L"plans\\DiaPlan3.inst"; // default plan file name
const wchar_t szDefaultPlanFile2[] = L"plans\\DiaMonitorPlan3.inst"; // default plan file name

#define MY_TIMER 1 // Timer ID

// Global Variables:
HINSTANCE hInst;								// current instance
unsigned int nRobots = 0;						// how many robots in the array
InstinctRobot *pMyRobots[MAX_ROBOTS];			// an array of pointers to robots
InstinctWorld theWorld;							// the world where the robots live
unsigned char bSuppressWarnings = FALSE;

// these variables contain global stats used for reporting
unsigned long ulTicks = 0;
unsigned long ulTotalRobotTicks = 0;
unsigned long ulMovingRobotTicks = 0;
unsigned long ulMonitorActiveTicks = 0;

// Forward declarations of functions included in this code module:
BOOL                InitInstance(HINSTANCE, int);
BOOL				ExitInstance(void);
INT_PTR CALLBACK	MainDialogProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
INT_PTR				MainCmdProc(HWND hDlg, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

BOOL AddRobotsToWorld(HWND hDlg, int nHowMany, wchar_t *pPlanFile, wchar_t *pPlanFile2, const unsigned char *pbMonitor, const unsigned char pbMonitorMon);
BOOL RemoveRobotsFromWorld(void);
BOOL ResetStats(HWND hDlg);
BOOL Tick(HWND hDlg);
BOOL ShowTotals(HWND hDlg);
BOOL ShowWorld(HWND hDlg);
void LoadPlan(HWND hDlg, Instinct::CmdPlanner *pPlan, Instinct::Names *pNames, wchar_t *pPlanFile, unsigned char *pSuppressWarnings);


// Entry point
int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	MSG msg;
	HWND hDlg;
	BOOL ret;

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}

	hDlg = CreateDialogParam(hInst, MAKEINTRESOURCE(IDD_DIALOG_MAIN), 0, MainDialogProc, 0);
	ShowWindow(hDlg, nCmdShow);

	while ((ret = GetMessage(&msg, 0, 0, 0)) != 0) {
		if (ret == -1)
			return -1;

		if (!IsDialogMessage(hDlg, &msg)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	ExitInstance();
	return 0;
}


// In this function, we save the instance handle in a global variable
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance; // Store instance handle in our global variable

	return TRUE;
}

// clean up before we exit
BOOL ExitInstance(void)
{
	RemoveRobotsFromWorld();

	return TRUE;
}

// make some new robots
BOOL AddRobotsToWorld(HWND hDlg, int nHowMany, wchar_t *pPlanFile, wchar_t *pPlanFile2, const unsigned char *pbMonitor, const unsigned char *pbMonitorMon)
{
	// make some robot log file names
	static char cName = 'a';
	static char fName = 'a';
	char szLogFileName[MAX_PATH];
	char szMonLogFileName[MAX_PATH];

	bSuppressWarnings = FALSE;

	for (int i = 0; i < nHowMany; i++)
	{
		if (nRobots >= MAX_ROBOTS) // we can't add any more robots
		{
			MessageBox(hDlg, L"Maximum of 100 robots added to the world", L"", MB_OK);
			break;
		}
		snprintf(szLogFileName, sizeof(szLogFileName), "logs\\RobLog-%c%c.txt", fName, cName);
		snprintf(szMonLogFileName, sizeof(szMonLogFileName), "logs\\RobMonLog-%c%c.txt", fName, cName);
		InstinctRobot *pRobot = new InstinctRobot(&theWorld, cName, szLogFileName, szMonLogFileName, NAMES_BUFFER_SIZE);

		Instinct::CmdPlanner *pPlan = pRobot->getPlan();
		Instinct::Names *pNames = pRobot->getNames();
		LoadPlan(hDlg, pPlan, pNames, pPlanFile, &bSuppressWarnings);
		pPlan->setGlobalMonitorFlags(pbMonitor[0], pbMonitor[1], pbMonitor[2], pbMonitor[3], pbMonitor[4], pbMonitor[5]);
		
		MonitorPlanWorld *pMonitorPlanWorld = pRobot->getMonitorPlanWorld();
		Instinct::CmdPlanner *pMonitorPlan = pMonitorPlanWorld->getPlan();
		LoadPlan(hDlg, pMonitorPlan, pMonitorPlanWorld->getNames(), pPlanFile2, &bSuppressWarnings);
		pMonitorPlan->setGlobalMonitorFlags(pbMonitorMon[0], pbMonitorMon[1], pbMonitorMon[2], pbMonitorMon[3], pbMonitorMon[4], pbMonitorMon[5]);


		// this is very specific to a plan with the Mate Drive
		// it sets the initial Runtime_Priority to a random value within range for the plan
		// this avoids initial bunching as the robots are all borne at once and ramp together
		Instinct::instinctID bMateDriveID = pNames->getElementID("Mate");
		Instinct::instinctID bExploreDriveID = pNames->getElementID("Explore");
		if (bMateDriveID && bExploreDriveID)
		{
			Instinct::instinctID nMatePriority = pPlan->getDrivePriority(bMateDriveID);
			Instinct::instinctID nRange = max(1, pPlan->getDrivePriority(bExploreDriveID) - nMatePriority);
			nMatePriority += (Instinct::instinctID)((((unsigned long)rand() * (unsigned long)nRange) / (unsigned long)RAND_MAX)/2);
			pPlan->setRuntimeDrivePriority(bMateDriveID, nMatePriority);
		}

		pMyRobots[nRobots] = pRobot;
		nRobots++;
		// a-z, A-Z, 0-9, then loop - gives 62 robot names
		if (cName == 'z')
		{
			cName = 'A';
			if (fName == 'z') // a file name 'A' is same as 'a'
				fName = 'a';
			else
				fName++; 
		}
		else if (cName == 'Z')
			cName = '0';
		else if (cName == '9')
		{
			cName = 'a';
			if (fName == 'z')
				fName = 'a';
			else
				fName++;
		}
		else
			cName++;
		// start the robots in different positions so we can see them
		for (int j = 0; j < i; j++)
			pRobot->executeAction(ACTION_TURN, 45, false);
	}

	return TRUE;
}

BOOL RemoveRobotsFromWorld(void)
{
	for (unsigned int i = 0; i < nRobots; i++)
	{
		InstinctRobot *pRobot = pMyRobots[i];
		theWorld.removeRobot(pRobot);
		delete pRobot;
	}
	nRobots = 0;

	return TRUE;
}

BOOL Tick(HWND hDlg)
{
	ulTicks++;

	SetDlgItemInt(hDlg, IDC_TOTAL_TICKS, ulTicks, FALSE);

	for (unsigned int i = 0; i < nRobots; i++)
	{
		InstinctRobot *pRobot = pMyRobots[i];
		pRobot->runPlan();
	}
	theWorld.tick();
	unsigned int uiMovingRobots = theWorld.movingRobots();
	SetDlgItemInt(hDlg, IDC_MOVING_ROBOTS, uiMovingRobots, FALSE);

	// update sums for stats
	ulTotalRobotTicks += nRobots;
	ulMovingRobotTicks += uiMovingRobots;
	unsigned int uiMonitorActive = 0;
	for (unsigned int i = 0; i < nRobots; i++)
	{
		uiMonitorActive += (pMyRobots[i]->getRobotChar() == '!') ? 1 : 0;
	}
	ulMonitorActiveTicks += uiMonitorActive;

	ShowTotals(hDlg);
	return ShowWorld(hDlg);
}

BOOL ResetStats(HWND hDlg)
{
	ulTicks = 0;
	ulTotalRobotTicks = 0;
	ulMovingRobotTicks = 0;
	ulMonitorActiveTicks = 0;

	for (unsigned int i = 0; i < nRobots; i++)
	{
		pMyRobots[i]->resetMatings();
	}

	return TRUE;
}

BOOL ShowTotals(HWND hDlg)
{
	// calculate average ticks per mating per robot
	// add up total matings, divide by number of robots, divide by ticks
	// also sum the robots affected by the Monitor Plan
	unsigned long ulTotalMatings = 0;
	unsigned int uiMonitorActive = 0;
	for (unsigned int i = 0; i < nRobots; i++)
	{
		ulTotalMatings += pMyRobots[i]->getMatings();
		uiMonitorActive += (pMyRobots[i]->getRobotChar() == '!') ? 1 : 0;
	}

	SetDlgItemInt(hDlg, IDC_TOTAL_TICKS, ulTicks, FALSE);
	SetDlgItemInt(hDlg, IDC_TOTAL_ROBOTS, nRobots, FALSE);
	SetDlgItemInt(hDlg, IDC_MOVING_ROBOTS, theWorld.movingRobots(), FALSE);

	SetDlgItemInt(hDlg, IDC_MONITOR_ACTIVE, uiMonitorActive, FALSE);
	SetDlgItemInt(hDlg, IDC_MOVING_ROBOTS_AVG, (ulMovingRobotTicks * 100) / max(1L, ulTotalRobotTicks), FALSE);
	SetDlgItemInt(hDlg, IDC_AVG_TICKS, ulTotalMatings ? (ulTicks * nRobots) / max(1L, ulTotalMatings) : 0, FALSE);
	SetDlgItemInt(hDlg, IDC_AVG_MONITOR_ACTIVE, (ulMonitorActiveTicks * 100) / max(1L, ulTotalRobotTicks), FALSE);

	return TRUE;
}

BOOL ShowWorld(HWND hDlg)
{
	if (hDlg && (BST_CHECKED == SendDlgItemMessage(hDlg, IDC_SHOW_WORLD, BM_GETCHECK, 0, 0)))
	{
		wchar_t szTextBuff[((WORLD_X + 2) * WORLD_Y)+1];
		theWorld.showWorld(szTextBuff, sizeof(szTextBuff)/sizeof(wchar_t));
		SetDlgItemText(hDlg, IDC_WORLD, (LPCWSTR)szTextBuff);
		return TRUE;
	}

	return FALSE;
}

// window proc for the main application window, which is a dialog box
INT_PTR CALLBACK MainDialogProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	HFONT hFont = 0;

	switch (uMsg)
	{
	case WM_INITDIALOG:
		SendDlgItemMessage(hDlg, IDC_SHOW_WORLD, BM_SETCHECK, BST_CHECKED, 0);
		SetDlgItemText(hDlg, IDC_PLANFILE, szDefaultPlanFile);
		SetDlgItemText(hDlg, IDC_PLANFILE2, szDefaultPlanFile2);
		SetDlgItemInt(hDlg, IDC_RATE, DEFAULT_RATE, FALSE);
		SetDlgItemInt(hDlg, IDC_ROBOTS, MAKE_ROBOTS, FALSE);
		hFont = (HFONT)GetStockObject(SYSTEM_FIXED_FONT);
		SendDlgItemMessage(hDlg, IDC_WORLD, WM_SETFONT, (WPARAM)hFont, 0L);
		ResetStats(hDlg);
		ShowTotals(hDlg);
		ShowWorld(hDlg);
		EnableWindow(GetDlgItem(hDlg, IDC_STOP), FALSE);
		SetFocus(GetDlgItem(hDlg, IDC_LOADPLAN));
		return TRUE;

	case WM_COMMAND:
		return MainCmdProc(hDlg, wParam, lParam);
		break;

	case WM_TIMER:
		if (wParam == MY_TIMER)
			Tick(hDlg);
		break;

	case WM_CLOSE:
		if (MessageBox(hDlg, TEXT("Close the program?"), TEXT("Close"),
			MB_ICONQUESTION | MB_YESNO) == IDYES)
		{
			KillTimer(hDlg, MY_TIMER);
			DestroyWindow(hDlg);
		}
		return TRUE;

	case WM_DESTROY:
		PostQuitMessage(0);
		return TRUE;
	}

	return FALSE;
}

// processes WM_COMMAND events
INT_PTR MainCmdProc(HWND hDlg, WPARAM wParam, LPARAM lParam)
{
	HWND hDlgAbout;
	int nRate;
	int nHowManyRobots;
	wchar_t szPlanFile[MAX_PATH];
	wchar_t szPlanFile2[MAX_PATH];
	unsigned char bMonitor[6];
	unsigned char bMonitorMon[6];

	switch (LOWORD(wParam))
	{
	case IDC_LOADPLAN:
		szPlanFile[0] = L'\0';
		GetDlgItemText(hDlg, IDC_PLANFILE, szPlanFile, sizeof(szPlanFile)/sizeof(wchar_t));
		GetDlgItemText(hDlg, IDC_PLANFILE2, szPlanFile2, sizeof(szPlanFile2) / sizeof(wchar_t));
		nHowManyRobots = GetDlgItemInt(hDlg, IDC_ROBOTS, NULL, FALSE);

		bMonitor[0] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_EXECUTE, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitor[1] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_SUCCESS, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitor[2] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_INPROGRESS, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitor[3] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_FAIL, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitor[4] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_ERROR, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitor[5] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_RELEASER, BM_GETSTATE, 0, 0L)) ? 1 : 0;

		bMonitorMon[0] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_EXECUTE2, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitorMon[1] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_SUCCESS2, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitorMon[2] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_INPROGRESS2, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitorMon[3] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_FAIL2, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitorMon[4] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_ERROR2, BM_GETSTATE, 0, 0L)) ? 1 : 0;
		bMonitorMon[5] = (BST_CHECKED & SendDlgItemMessage(hDlg, IDC_MON_RELEASER2, BM_GETSTATE, 0, 0L)) ? 1 : 0;

		// RemoveRobotsFromWorld();
		AddRobotsToWorld(hDlg, min(nHowManyRobots, MAX_ROBOTS), szPlanFile, szPlanFile2, bMonitor, bMonitorMon);
		ShowTotals(hDlg);
		ShowWorld(hDlg);
		SetDlgItemInt(hDlg, IDC_TOTAL_ROBOTS, nRobots, FALSE);
		return TRUE;

	case IDC_START:
		nRate = GetDlgItemInt(hDlg, IDC_RATE, NULL, FALSE);
		SetTimer(hDlg, MY_TIMER, 1000/(nRate ? nRate : 1), NULL);
		EnableWindow(GetDlgItem(hDlg, IDC_START), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_STOP), TRUE);
		return TRUE;

	case IDC_STOP:
		KillTimer(hDlg, MY_TIMER);
		EnableWindow(GetDlgItem(hDlg, IDC_START), TRUE);
		EnableWindow(GetDlgItem(hDlg, IDC_STOP), FALSE);
		return TRUE;

	case IDC_STEP:
		Tick(hDlg);
		return TRUE;

	case IDC_RESET:
		KillTimer(hDlg, MY_TIMER);
		RemoveRobotsFromWorld();
		ResetStats(hDlg);
		ShowTotals(hDlg);
		ShowWorld(hDlg);
		EnableWindow(GetDlgItem(hDlg, IDC_START), TRUE);
		EnableWindow(GetDlgItem(hDlg, IDC_STOP), FALSE);
		return TRUE;

	case IDC_RESET2:
		ResetStats(hDlg);
		ShowTotals(hDlg);
		return TRUE;

	case IDC_SHOW_WORLD:
		if (BST_CHECKED == SendDlgItemMessage(hDlg, IDC_SHOW_WORLD, BM_GETCHECK, 0, 0))
		{
			ShowWorld(hDlg);
		}
		break;

	case IDC_ABOUT:
		hDlgAbout = CreateDialogParam(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hDlg, About, 0);
		ShowWindow(hDlgAbout, true);
		return TRUE;
	}
	return FALSE;
}


// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}


void LoadPlan(HWND hDlg, Instinct::CmdPlanner *pPlan, Instinct::Names *pNames, wchar_t *pPlanFile, unsigned char *pSuppressWarnings)
{
	char szMsgBuff[100];
	HANDLE hFile;

	// ignore blank file name or null pointers
	if (!pPlanFile || !lstrlenW(pPlanFile) || !pPlan || !pNames || !pSuppressWarnings)
		return;

	hFile = CreateFile(pPlanFile, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
	if (hFile == INVALID_HANDLE_VALUE)
	{
		if (!*pSuppressWarnings && MessageBox(hDlg, pPlanFile, TEXT("Invalid Plan File Name"),
			MB_ICONWARNING | MB_OKCANCEL) == IDCANCEL)
			*pSuppressWarnings = TRUE;
	}
	else
	{
		DWORD dwFileSize = GetFileSize(hFile, NULL);
		if (dwFileSize != 0xFFFFFFFF)
		{
			LPSTR pszFileText;

			pszFileText = (LPSTR)GlobalAlloc(GPTR, dwFileSize + 1);
			if (pszFileText != NULL)
			{
				DWORD dwRead;

				if (ReadFile(hFile, pszFileText, dwFileSize, &dwRead, NULL))
				{
					*(pszFileText+dwRead) = 0; // Add null terminator
					// now we have the whole file in RAM

					LPSTR pTxt = pszFileText;
					LPSTR pLine = pTxt;

					wchar_t szWideChar[120];


					while (*pTxt && *pLine)
					{
						if ((*pTxt == '\r') || (*pTxt == '\n'))
						{
							*pTxt = 0; // zero terminate the string
							if (strlen(pLine) && !strstr(pLine, "//"))
							{
								char szCmd[80];
								sscanf(pLine, "%s", szCmd);

								if (!_strcmpi(szCmd, "PLAN") && (strlen(pLine) > 5))
								{
									if (!(pPlan->executeCommand(pLine + 5, szMsgBuff, sizeof(szMsgBuff))))
									{
										MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, pLine, -1, szWideChar, sizeof(szWideChar) / sizeof(wchar_t));
										MessageBox(hDlg, szWideChar, L"Error Executing Command", MB_OK);
									}
								}
								else if (!_strcmpi(szCmd, "PELEM") && (strlen(pLine) > 6))
								{
									char *pEq = strchr(pLine + 6, '=');
									if (pEq)
									{
										*pEq = ' ';
									}
									char szName[20];
									unsigned int uiID;
									if (sscanf(pLine+6, "%s %u", szName, &uiID) == 2)
									{
										// add the name and ID pair to the names buffer
										pNames->addElementName(uiID, szName);
									}
								}
							}
							pLine = pTxt + 1;
						}
						pTxt++;
					}

				}
				GlobalFree(pszFileText);
			}
		}
		CloseHandle(hFile);
	}
}

