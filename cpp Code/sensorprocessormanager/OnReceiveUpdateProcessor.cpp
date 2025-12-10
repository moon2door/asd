#include <iostream>
#include <vector>
#include <afxwin.h>
#include <tlhelp32.h>

#include "stdafx.h"
#include "SensorProcessorManager.h"
#include "SensorProcessorManagerDlg.h"

BOOL CSensorProcessorManagerDlg::SafeTerminateProcess(SHELLEXECUTEINFO& sei)
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

void CSensorProcessorManagerDlg::SafeTerminateAllProcess()
{
	std::vector<DWORD> pids = GetProcessIDsByName({ L"ClusterProcessor.exe", L"interfacerotor.exe", L"SensorProcessorManager.exe" });

	for (int i = 0; i < pids.size(); i++)
	{
		HANDLE hProcess = OpenProcess(PROCESS_TERMINATE, FALSE, pids[i]);
		if (hProcess == NULL) {
			std::cerr << "Failed to open process. Error code: " << GetLastError() << std::endl;
		}

		if (!TerminateProcess(hProcess, 0)) {
			std::cerr << "Failed to terminate process. Error code: " << GetLastError() << std::endl;
		}

		CloseHandle(hProcess);
	}
}
//

bool CSensorProcessorManagerDlg::UpdateProcessor(SHELLEXECUTEINFO& sei, uint8_t* pData, uint32_t size)
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

void CSensorProcessorManagerDlg::RunProcessor(SHELLEXECUTEINFO& sei)
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

void CSensorProcessorManagerDlg::OnUpdateSensorManagerProc(uint8_t* pUpdateData, uint32_t size)
{
	CFile file;
	if (file.Open(".\\_temp_", CFile::modeWrite | CFile::modeCreate | CFile::typeBinary))
	{
		file.SeekToBegin();
		file.Write(pUpdateData, size);
		file.Flush();
		file.Close();

		SafeTerminateProcess(m_seiInterface);
		SafeTerminateProcess(m_seiCluster);

		RunProcessor(m_seiSensorManager);
	}
}

void CSensorProcessorManagerDlg::OnUpdateInterfaceProc(uint8_t* pUpdateData, uint32_t size)
{
	UpdateProcessor(m_seiInterface, pUpdateData, size);
	RunProcessor(m_seiInterface);
}

void CSensorProcessorManagerDlg::OnUpdateClusterProc(uint8_t* pUpdateData, uint32_t size)
{
	UpdateProcessor(m_seiCluster, pUpdateData, size);
	RunProcessor(m_seiCluster);
}

