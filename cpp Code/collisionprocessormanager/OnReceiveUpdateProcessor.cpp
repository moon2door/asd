#include <vector>
#include <afxwin.h>
#include <tlhelp32.h>

#include "stdafx.h"
#include "CollisionProcessorManager.h"
#include "CollisionProcessorManagerDlg.h"

BOOL CCollisionProcessorManagerDlg::SafeTerminateProcess(SHELLEXECUTEINFO& sei)
{
	DWORD dwTID, dwCode, dwErr = 0;
	HANDLE hProcessDup = INVALID_HANDLE_VALUE;
	HANDLE hRT = NULL;
	HINSTANCE hKernel = GetModuleHandle("Kernel32");

	BOOL bSuccess = FALSE;
	BOOL bDup = DuplicateHandle(GetCurrentProcess(),
		sei.hProcess,
		GetCurrentProcess(),
		&hProcessDup,
		PROCESS_ALL_ACCESS,
		FALSE,
		0);
	if (GetExitCodeProcess((bDup) ? hProcessDup : sei.hProcess, &dwCode)
		&& (dwCode == STILL_ACTIVE))
	{
		FARPROC pfnExitProc;
		pfnExitProc = GetProcAddress(hKernel, "ExitProcess");
		hRT = CreateRemoteThread((bDup) ? hProcessDup : sei.hProcess,
			NULL,
			0,
			(LPTHREAD_START_ROUTINE)pfnExitProc,
			(PVOID)0, 0, &dwTID);
		if (hRT == NULL) dwErr = GetLastError();
	}
	else
	{
		dwErr = ERROR_PROCESS_ABORTED;
	}
	if (hRT)
	{
		WaitForSingleObject((bDup) ? hProcessDup : sei.hProcess, INFINITE);
		CloseHandle(hRT);
		bSuccess = TRUE;
	}
	if (bDup)
		CloseHandle(hProcessDup);
	if (!bSuccess)
		SetLastError(dwErr);

	if (::WaitForSingleObject(sei.hProcess, 5000) == WAIT_OBJECT_0)
	{
		CloseHandle(sei.hwnd);
		sei.hProcess = NULL;
		sei.hwnd = NULL;

		bSuccess = TRUE;
	}
	else
	{
		bSuccess = FALSE;
	}

	return bSuccess;
}

//pjh
std::vector<DWORD> GetProcessIDsByName(const std::vector<std::wstring>& processNames) {
	std::vector<DWORD> processIDs;

	for (const auto& processName : processNames) {
		PROCESSENTRY32 entry;
		entry.dwSize = sizeof(PROCESSENTRY32);

		HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);

		if (Process32First(snapshot, &entry) == TRUE) {
			while (Process32Next(snapshot, &entry) == TRUE) {
				wchar_t szExeFile[MAX_PATH];
				MultiByteToWideChar(CP_ACP, 0, entry.szExeFile, -1, szExeFile, MAX_PATH);
				if (_wcsicmp(szExeFile, processName.c_str()) == 0) {
					processIDs.push_back(entry.th32ProcessID);
				}
			}
		}

		CloseHandle(snapshot);
	}

	return processIDs;
}

void CCollisionProcessorManagerDlg::SafeTerminateAllProcess()
{
	std::vector<DWORD> pids = GetProcessIDsByName({
		L"PLCProcessor.exe",
		L"DistanceProcessor.exe",
		L"CollisionProcessor.exe",
		L"CollisionProcessorManager.exe"
	});

	for (int i = 0; i < pids.size(); i++)
	{
		HANDLE hProcess = OpenProcess(PROCESS_TERMINATE, FALSE, pids[i]);
		if (hProcess == NULL) {
			std::cerr << "Failed to open process. Error code: " << GetLastError() << std::endl;
			continue;
		}

		if (!TerminateProcess(hProcess, 0)) {
			std::cerr << "Failed to terminate process. Error code: " << GetLastError() << std::endl;
		}

		CloseHandle(hProcess);
	}
}
//

bool CCollisionProcessorManagerDlg::UpdateProcessor(SHELLEXECUTEINFO& sei, unsigned char* pData, uint32_t size)
{
	bool bRet = false;
	if (sei.hProcess)
	{
		if (SafeTerminateProcess(sei))
		{
			DeleteFile(sei.lpFile);

			CFile file;
			if (file.Open(sei.lpFile, CFile::modeWrite | CFile::modeCreate | CFile::typeBinary))
			{
				file.SeekToBegin();
				file.Write(pData, size);
				file.Flush();
				file.Close();
			}
		}
	}
	else
	{
		DeleteFile(sei.lpFile);

		CFile file;
		if (file.Open(sei.lpFile, CFile::modeWrite | CFile::modeCreate | CFile::typeBinary))
		{
			file.SeekToBegin();
			file.Write(pData, size);
			file.Flush();
			file.Close();
		}
	}

	return bRet;
}

void CCollisionProcessorManagerDlg::RunProcessor(SHELLEXECUTEINFO& sei)
{
	if (sei.hProcess == NULL)
	{
		DWORD result = ::ShellExecuteEx(&sei);
	}
	else
	{
		SafeTerminateProcess(sei);
	}
}

void CCollisionProcessorManagerDlg::OnUpdateSensorManagerProc(unsigned char* pUpdateData, uint32_t size)
{
	m_objSensorManager->SendUpdateSensorManagerProc(pUpdateData, size);
}

void CCollisionProcessorManagerDlg::OnUpdateInterfaceProc(unsigned char* pUpdateData, uint32_t size)
{
	m_objSensorManager->SendUpdateInterfaceProc(pUpdateData, size);
}

void CCollisionProcessorManagerDlg::OnUpdateClusterProc(unsigned char* pUpdateData, uint32_t size)
{
	m_objSensorManager->SendUpdateClusterProc(pUpdateData, size);
}

void CCollisionProcessorManagerDlg::OnUpdateCollisionManagerProc(unsigned char* pUpdateData, uint32_t size)
{
	CFile file;
	if (file.Open(".\\_temp_", CFile::modeWrite | CFile::modeCreate | CFile::typeBinary))
	{
		file.SeekToBegin();
		file.Write(pUpdateData, size);
		file.Flush();
		file.Close();

		SafeTerminateProcess(m_seiDistance);
		SafeTerminateProcess(m_seiCollision);
		SafeTerminateProcess(m_seiPLC);

		RunProcessor(m_seiCollisionManager);
	}
}

void CCollisionProcessorManagerDlg::OnUpdateDistanceProc(unsigned char* pUpdateData, uint32_t size)
{
	UpdateProcessor(m_seiDistance, pUpdateData, size);
	RunProcessor(m_seiDistance);
}

void CCollisionProcessorManagerDlg::OnUpdateCollisionProc(unsigned char* pUpdateData, uint32_t size)
{
	UpdateProcessor(m_seiCollision, pUpdateData, size);
	RunProcessor(m_seiCollision);
}

void CCollisionProcessorManagerDlg::OnUpdatePLCProc(unsigned char* pUpdateData, uint32_t size)
{
	UpdateProcessor(m_seiPLC, pUpdateData, size);
	RunProcessor(m_seiPLC);
}

void CCollisionProcessorManagerDlg::OnUpdateMonitoring(unsigned char* pUpdateData, uint32_t size)
{
	m_craneMonitoringInterface->SendUpdateMonitoring(pUpdateData, size);
}

