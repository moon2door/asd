#include "Cluster.h"
#include <list>
#include <pcl/common/distances.h>

namespace SHI
{
	namespace Cluster
	{
		bool IsNearCluster(const PointCloud& cloud, const Indices& indices1, const Indices& indices2, float distance)
		{
			bool ret = false;
			float distanceSq = distance*distance;

			if (indices1.size() > 0 && indices2.size() > 0)
			{
				// 기준 클러스터 선택 (갯수가 적은 클러스터)
				Indices i1, i2;
				if (indices1.size() > indices2.size())
				{
					i1 = indices2;
					i2 = indices1;
				}
				else
				{
					i2 = indices2;
					i1 = indices1;
				}

				// 기준 클러스터의 바운더리 계산
				pcl::PointXYZ pMin(FLT_MAX, FLT_MAX, FLT_MAX), pMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);
				for (unsigned int i = 0; i < i1.size(); i++)
				{
					pcl::PointXYZ p = cloud.points.at(i1.at(i));
					pMin.x = std::min(p.x, pMin.x);
					pMin.y = std::min(p.y, pMin.y);
					pMin.z = std::min(p.z, pMin.z);
					pMax.x = std::max(p.x, pMax.x);
					pMax.y = std::max(p.y, pMax.y);
					pMax.z = std::max(p.z, pMax.z);
				}
				pMin.x -= (distance + 1.0); // 1.0 : margin
				pMin.y -= (distance + 1.0); // 1.0 : margin
				pMin.z -= (distance + 1.0); // 1.0 : margin
				pMax.x += (distance + 1.0); // 1.0 : margin
				pMax.y += (distance + 1.0); // 1.0 : margin
				pMax.z += (distance + 1.0); // 1.0 : margin

				// 바운더리 내의 비교 클러스터 추출
				pcl::PointCloud<pcl::PointXYZ> pointsInBoundary;
				for (unsigned int i = 0; i < i2.size(); i++)
				{
					pcl::PointXYZ p = cloud.points.at(i2.at(i));
					if (pMin.x < p.x && p.x < pMax.x &&
						pMin.y < p.y && p.y < pMax.y &&
						pMin.z < p.z && p.z < pMax.z)
					{
						pointsInBoundary.push_back(p);
					}
				}

				if (pointsInBoundary.size() > 0)
				{
					// 최소 거리 비교
					bool bExecute = false;
					for (unsigned int i = 0; i < i1.size() && !bExecute; i++)
					{
						for (unsigned int j = 0; j < pointsInBoundary.size() && !bExecute; j++)
						{
							pcl::PointXYZ p1 = cloud.points.at(i1.at(i));
							pcl::PointXYZ p2 = pointsInBoundary.points.at(j);

							if (distanceSq > pcl::squaredEuclideanDistance(p1, p2))
							{
								ret = true;
								bExecute = true;
							}
						}
					}
				}
			}

			return ret;
		}

		void _ClusterVectors(std::list<std::vector<int>> &data)
		{
			int totalSize = (int)data.size();
			if (totalSize > 2)
			{
				std::list<std::vector<int>>::iterator itCur, itCompare;
				for (itCur = data.begin(); itCur != data.end(); ++itCur)
				{
					for (itCompare = itCur; itCompare != data.end(); ++itCompare)
					{
						if (itCompare != itCur)
						{
							// 두 벡터에 같은 데이터가 있는지 확인
							bool bSameCluster = false;
							for (int i = 0; i < itCur->size() && !bSameCluster; i++)
							{
								for (int j = 0; j < itCompare->size() && !bSameCluster; j++)
								{
									if (itCur->at(i) == itCompare->at(j))
									{
										bSameCluster = true;
									}
								}
							}

							if (bSameCluster)
							{
								// 기준 벡터에 비교 벡터 추가
								//itCur->insert(itCur->end(), itCompare->begin(), itCompare->end());
								for (unsigned int i = 0; i < itCompare->size(); i++)
								{
									// 기존 벡터에 있는지 검사
									bool bExists = false;
									for (unsigned int j = 0; j < itCur->size() && !bExists; j++)
									{
										if (itCompare->at(i) == itCur->at(j))
										{
											bExists = true;
										}
									}
									// 기존 벡터에 없을때만 추가함
									if (!bExists)
									{
										itCur->push_back(itCompare->at(i));
									}
								}

								// 비교 벡터 삭제
								itCompare = data.erase(itCompare);
								itCompare--;
							}
						}
					}
				}
			}
		}

		void ClusterVectors(std::list<std::vector<int>> &src, std::list<std::vector<int>> &dst)
		{
			std::list<std::vector<int>> tmp = src;

			unsigned int maxloop = src.size();
			int s1 = tmp.size();
			_ClusterVectors(tmp);
			int s2 = tmp.size();

			unsigned int count = 0;
			while (s1 != s2 && count++ < maxloop)
			{
				s1 = tmp.size();
				_ClusterVectors(tmp);
				s2 = tmp.size();
			}
			dst = tmp;
		}

		void MergeClusters(
			PointCloud& cloudNew,
			PCLIndicesVector& clusterIndicesNew,
			PointCloudPtr cloudFilteredSub1,
			PCLIndicesVectorPtr clusterIndicesSub1,
			PointCloudPtr cloudFilteredSub2,
			PCLIndicesVectorPtr clusterIndicesSub2,
			float distance)
		{
			// 클러스터링 결과 Merge
			PCLIndicesVector indicesTmp;

			cloudNew += *cloudFilteredSub1;
			cloudNew += *cloudFilteredSub2;

			for (unsigned int i = 0; i < clusterIndicesSub2->size(); i++)
			{
				for (unsigned int j = 0; j < clusterIndicesSub2->at(i).indices.size(); j++)
				{
					clusterIndicesSub2->at(i).indices.at(j) += cloudFilteredSub1->size();
				}
			}
			indicesTmp.insert(indicesTmp.end(), clusterIndicesSub1->begin(), clusterIndicesSub1->end());
			indicesTmp.insert(indicesTmp.end(), clusterIndicesSub2->begin(), clusterIndicesSub2->end());

			// 인접 클러스터 병합 처리
			if (indicesTmp.size() > 0)
			{
				std::list<Indices> listClusters, listClustersDst;
				unsigned int a = 0, b = 0;

				// 클러스터 순회
				for (int idxSrc = 0; idxSrc < indicesTmp.size() - 1; idxSrc++)
				{
					for (int idxDst = idxSrc + 1; idxDst < indicesTmp.size(); idxDst++)
					{
						PCLIndices* i1 = &indicesTmp.at(idxSrc);
						PCLIndices* i2 = &indicesTmp.at(idxDst);

						// 기준 거리로 인접 클러스터 여부 판단
						if (IsNearCluster(cloudNew, i1->indices, i2->indices, distance))
						{
							// 인접 클러스터
							// 인접하면 두 클러스터 번호를 한 벡터에 넣음
							Indices indicesNew;
							indicesNew.push_back(idxSrc);
							indicesNew.push_back(idxDst);

							listClusters.push_back(indicesNew);
						}
						else
						{
							// 인접하지 않은 클러스터
							// 인접하지 않으면 두 클러스터 번호를 각각의 벡터에 넣음
							Indices indicesNew1, indicesNew2;
							indicesNew1.push_back(idxSrc);
							indicesNew2.push_back(idxDst);

							listClusters.push_back(indicesNew1);
							listClusters.push_back(indicesNew2);
						}
					}
				}

				// 인접 클러스터 정보 정리
				// 앞서서 인접 클러스터 쌍으로 정리된 번호들을 재정리
				// (1,2),(2,3),(3,4),(5) => (1,2,3,4), (5)와 같이 정리
				ClusterVectors(listClusters, listClustersDst);

				// 인접 클러스터 정보 처리
				// 반환 데이터 정리
				std::list<Indices>::iterator it = listClustersDst.begin();
				for (; it != listClustersDst.end(); ++it)
				{
					SHI::PCLIndices indicesAccum;
					for (unsigned int i = 0; i < it->size(); i++)
					{
						unsigned int idx = it->at(i);
						SHI::PCLIndices indices = indicesTmp.at(idx);
						indicesAccum.indices.insert(indicesAccum.indices.end(), indices.indices.begin(), indices.indices.end());
					}
					clusterIndicesNew.push_back(indicesAccum);
				}
			}
		}
	}
}