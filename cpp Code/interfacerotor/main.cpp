#include <conio.h>
#include <Routine/include/Routines/CProcess.h>
#include <Routine/include/Base/CThread.h>
#include <Routine/include/Base/RoutineUtility.h>
#include "InterfaceRotor.h"

class CMainClass
{
public:
	CMainClass(int32_t argc, char* argv[])
	{
		interfaceRotor.Create(argc);

		threadKeyboard.StartThread(&CMainClass::ProcessKeyboard, this);

		ProcessLog(argc, argv);
	}

	void ProcessKeyboard();

	void ProcessLog(int32_t argc, char* argv[]);

	SHI::CInterfaceRotor interfaceRotor;

	Routine::CThread threadKeyboard;
};


int32_t main(int32_t argc, char* argv[])
{
	HANDLE hHandle = NULL;
	hHandle = ::CreateMutex(NULL, TRUE, "InterfaceRotor");
	if (::GetLastError() == ERROR_ALREADY_EXISTS)
	{
		::CloseHandle(hHandle);
		return 0;
	}

	auto mainClass = CMainClass(argc, argv);

	auto& process = Routine::CProcess::Instance();
	process.Init();
	return process.Run();
}

void CMainClass::ProcessKeyboard()
{
	const auto currentThread = Routine::CThread::GetCurrentThread();
	while (currentThread->IsRunThread())
	{
		if (_kbhit())
		{
			switch (_getch())
			{
			case '1':
				interfaceRotor.SetRotation(0, true);
				break;
			case '!':
				interfaceRotor.SetRotation(0, false);
				break;
			case '2':
				interfaceRotor.SetRotation(1, true);
				break;
			case '@':
				interfaceRotor.SetRotation(1, false);
				break;
			case '3':
				interfaceRotor.SetRotation(2, true);
				break;
			case '#':
				interfaceRotor.SetRotation(2, false);
				break;
			case '4':
				interfaceRotor.SetRotation(3, true);
				break;
			case '$':
				interfaceRotor.SetRotation(3, false);
				break;
			case '5':
				interfaceRotor.SetRotation(4, true);
				break;
			case '%':
				interfaceRotor.SetRotation(4, false);
				break;
			case 'v':
				interfaceRotor.SetVisualize(true);
				break;
			case 'V':
				interfaceRotor.SetVisualize(false);
				break;
			default:
				break;
			}
		}
	}
}

void CMainClass::ProcessLog(int32_t argc, char* argv[])
{
	if (argc > 1)
	{
		//pjh
		std::vector<std::string> fileNames;
		std::vector<uint32_t> buffer;
		FILE* fp{};
		int64_t size = 0;
		long currentPosition;

		for (int32_t i = 1; i < argc; i++)
		{
			fileNames.emplace_back(argv[i]);
		}

		for (const auto& fileName : fileNames)
		{
			if (fopen_s(&fp, fileName.c_str(), "rb") == 0) 
			{
				//printf("Read log file: %s\n", fileName.c_str());
				break;
			}
		}

		fileNames.clear();
		while (fp != nullptr)
		{
			currentPosition = ftell(fp);

			while (fread(&size, 1, sizeof(size), fp) == sizeof(size))
			{
				if (size > 0)
				{
					buffer.resize(size);

					if (fread(buffer.data(), 1, size, fp) == size)
					{
						Routine::CDeSerializer deSerializer(buffer.data());

						Pala720Packet packet;
						deSerializer >> packet;
						interfaceRotor.OnRotorData(packet.id, &packet.rotorPacket);
						interfaceRotor.OnLidarPacket(packet.id, packet.lidarPacket);
					}
					else
					{
						printf("Failed to Read data from log file\n");
						break;
					}
				}
			}
			if (feof(fp))
			{
				clearerr(fp);
				rewind(fp);
			}
		}
		if(fp != nullptr)fclose(fp);  // NOLINT(cert-err33-c)
		buffer.clear();


		/*std::vector<std::string> fileNames;
		for (int32_t i = 1; i < argc; i++)
		{
			fileNames.emplace_back(argv[i]);
		}*/
		//for (const auto& fileName : fileNames)
		//{
		//	FILE* fp = nullptr;
		//	if (fopen_s(&fp, fileName.c_str(), "rb") == 0)
		//	{
		//		printf("Read log file: %s\n", fileName.c_str());

		//		int64_t size = 0;
		//		while (sizeof(size) == fread(&size, 1, sizeof(size), fp))
		//		{
		//			if (size > 0)
		//			{
		//				std::vector<uint32_t> buffer;
		//				buffer.resize(size);
		//				int64_t remained = size;
		//				while (remained > 0)
		//				{
		//					const auto nRead = fread(buffer.data(), 1, remained, fp);
		//					remained -= static_cast<int64_t>(nRead);
		//				}

		//				Routine::CDeSerializer deSerializer(buffer.data());

		//				Pala720Packet packet;
		//				deSerializer >> packet;
		//				interfaceRotor.OnRotorData(packet.id, &packet.rotorPacket);
		//				interfaceRotor.OnLidarPacket(packet.id, packet.lidarPacket);
		//			}
		//			else
		//			{
		//				Sleep(1000);
		//			}
		//		}

		//		fclose(fp);  // NOLINT(cert-err33-c)
		//	}
		//
	}
}