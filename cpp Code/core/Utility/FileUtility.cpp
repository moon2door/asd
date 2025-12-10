#include "FileUtility.h"
#include <windows.h>
#include <cstdio>
#include "../Data/StCluster.h"

namespace SHI
{
	namespace FileUtility
	{

		bool compare(const FileNameTime &a, const FileNameTime &b)
		{
			return CompareFileTime(&a.second, &b.second) < 0;
		}

		// path에 있는 파일 중 확장자가 extrnsion인 파일 목록 반환
		bool FindFiles(std::vector<std::string> &files, const char* path, const char* extension)
		{
			bool ret = false;
			Routine::char_t _path[256] = "";
			const int32_t nc = sprintf_s(_path, "%s/*.%s", path, extension);
			if(nc > 0)
			{
				WIN32_FIND_DATA findData;
				const HANDLE hFind = FindFirstFile(_path, &findData);
				if (hFind != INVALID_HANDLE_VALUE)
				{
					do
					{
						ret = true;
						std::string str = std::string(path) + '/' + std::string(findData.cFileName);
						files.push_back(str);
					} while (FindNextFile(hFind, &findData));
				}
				FindClose(hFind);
			}			
			return ret;
		}

		bool FindFiles(std::vector<FileNameTime> &files, const char* path, const char* extension)
		{
			bool ret = false;
			char _path[256] = "";
			const int32_t nc = sprintf_s(_path, "%s/*.%s", path, extension);
			if (nc > 0)
			{
				WIN32_FIND_DATA findData;
				const HANDLE hFind = FindFirstFile(_path, &findData);
				if (hFind != INVALID_HANDLE_VALUE)
				{
					do
					{
						ret = true;
						std::string str = std::string(path) + '/' + std::string(findData.cFileName);
						files.emplace_back(FileNameTime(str, findData.ftCreationTime));
					} while (FindNextFile(hFind, &findData));
				}
				FindClose(hFind);
			}
			return ret;
		}

		bool SaveClusterFile(SHI::Data::StCluster& cluster, const std::string &fname)
		{
			bool ret = false;
			FILE* fp = nullptr;
			if (fopen_s(&fp, fname.c_str(), "wb") == 0)
			{
				constexpr size_t headSize = sizeof(SHI::Data::StCluster) - SHI::Data::StCluster::_vMaxSize;
				size_t nWrite = 0;
				nWrite += fwrite(&cluster, 1, headSize, fp);
				nWrite += fwrite(cluster.vmem, 1, cluster._vSize, fp);
				fclose(fp);  // NOLINT(cert-err33-c)
				if (nWrite == cluster.GetSize())
				{
					ret = true;
				}
			}
			return ret;
		}

		bool LoadClusterFile(SHI::Data::StCluster& cluster, const std::string &fname)
		{
			bool ret = false;
			FILE* fp = nullptr;
			if (fopen_s(&fp, fname.c_str(), "rb") == 0)
			{
				constexpr size_t headSize = sizeof(SHI::Data::StCluster) - SHI::Data::StCluster::_vMaxSize;
				const size_t nRead = fread(&cluster, 1, headSize, fp);
				if (nRead == headSize)
				{
					fread(cluster.vmem, 1, cluster._vSize, fp);  // NOLINT(cert-err33-c)
					ret = true;
				}
				fclose(fp);  // NOLINT(cert-err33-c)
			}
			return ret;
		}

		bool SaveDistanceFile(SHI::Data::StDistance& distance, const std::string &fname)
		{
			bool ret = false;
			FILE* fp = nullptr;
			if (fopen_s(&fp, fname.c_str(), "wb") == 0)
			{
				constexpr size_t headSize = sizeof(SHI::Data::StDistance) - SHI::Data::StDistance::_vMaxSize;
				size_t nWrite = 0;
				nWrite += fwrite(&distance, 1, headSize, fp);
				nWrite += fwrite(distance.vmem, 1, distance._vSize, fp);
				fclose(fp);  // NOLINT(cert-err33-c)
				if(nWrite == distance.GetSize())
				{
					ret = true;
				}
			}
			return ret;
		}

		bool LoadDistanceFile(SHI::Data::StDistance& distance, const std::string &fname)
		{
			bool ret = false;
			FILE* fp = nullptr;
			if (fopen_s(&fp, fname.c_str(), "rb") == 0)
			{
				constexpr size_t headSize = sizeof(SHI::Data::StDistance) - SHI::Data::StDistance::_vMaxSize;
				const size_t nRead = fread(&distance, 1, headSize, fp);
				if(nRead == headSize)
				{
					fread(distance.vmem, 1, distance._vSize, fp);  // NOLINT(cert-err33-c)
					ret = true;
				}				
				fclose(fp);  // NOLINT(cert-err33-c)
			}
			return ret;
		}

	}
}
