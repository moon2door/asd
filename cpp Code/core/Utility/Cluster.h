#pragma once

#include "Types.h"

namespace SHI
{
	namespace Cluster
	{
		bool IsNearCluster(const PointCloud& cloud, const Indices& indices1, const Indices& indices2, float distance);

		void MergeClusters(
			PointCloud& cloudNew,
			PCLIndicesVector& clusterIndicesNew,
			PointCloudPtr cloudFilteredSub1,
			PCLIndicesVectorPtr clusterIndicesSub1,
			PointCloudPtr cloudFilteredSub2,
			PCLIndicesVectorPtr clusterIndicesSub2,
			float distance);
	};
}