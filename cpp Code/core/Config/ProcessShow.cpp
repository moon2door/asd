#include "ProcessShow.h"
#include "colorTable.h"
#include <pcl/visualization/pcl_visualizer.h>

CProcessShow::CProcessShow()
	: m_cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
	m_eData.Create();
}

void CProcessShow::StartProcessShow()
{
	m_thread.StartThread(&CProcessShow::ThreadVisualize, this);
}

void CProcessShow::StopProcessShow()
{
	m_thread.StopThread();
}

void CProcessShow::UpdateShowXyz(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
	if(m_thread.IsRunThread())
	{
		m_csData.Lock();
		m_cloud = cloud;
		m_csData.UnLock();
		// ReSharper disable once CppExpressionWithoutSideEffects
		m_eData.SetEvent();
	}
}

void CProcessShow::ThreadVisualize() const
{
	pcl::visualization::PCLVisualizer viewer("test viewer");
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudShow(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloudShow);
	viewer.addCoordinateSystem(10);

	while (m_thread.IsRunThread())
	{
		if (m_eData.WaitForEvent(100))
		{
			m_csData.Lock();
			const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_cloud;
			m_csData.UnLock();

			cloudShow->clear();

			//float maxVal = -FLT_MAX;
			//for (unsigned int i = 0; i < cloud->size(); i++)
			//{
			//	maxVal = (std::max)(maxVal, cloud->points[i].intensity);
			//}

			//for (unsigned int i=0; i<cloud->size(); i++)
			//{
			//	MyColor::Color color = MyColor::GetTable(cloud->points[i].intensity, 0, maxVal);
			//	pcl::PointXYZRGB p;
			//	p.x = cloud->points[i].x;
			//	p.y = cloud->points[i].y;
			//	p.z = cloud->points[i].z;
			//	p.r = color.r;
			//	p.g = color.g;
			//	p.b = color.b;
			//	cloudShow->push_back(p);
			//}

			for (size_t i = 0; i < cloud->size(); i++)
			{
				MyColor::Color color;
				if (fabs(cloud->points[i].intensity) < FLT_EPSILON)
				{
					color = MyColor::Red;
				}
				else if (fabs(cloud->points[i].intensity - 1.0f) < FLT_EPSILON)
				{
					color = MyColor::Green;
				}
				else
				{
					color = MyColor::Blue;
				}
				pcl::PointXYZRGB p;
				p.x = cloud->points[i].x;
				p.y = cloud->points[i].y;
				p.z = cloud->points[i].z;
				p.r = color.r;
				p.g = color.g;
				p.b = color.b;
				cloudShow->push_back(p);
			}
			viewer.removePointCloud();
			viewer.addPointCloud<pcl::PointXYZRGB>(cloudShow);
		}

		viewer.spinOnce();
	}
}
