
#include "DrawUtility.h"
#include "colorTable.h"
#include "Transform.h"

namespace SHI
{
	namespace Utility
	{
		static PointCloudPtr g_emptyPoint = PointCloudPtr(new PointCloud);
		static pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_emptyPointRgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		void AddRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr roi, float r, float g, float b, const const char* name)
		{
			if (roi->size() == 8)
			{
				char _name[256] = "";
				sprintf(_name, "%s_01", name);
				viewer->addLine(roi->points[0], roi->points[1], r, g, b, _name);
				sprintf(_name, "%s_02", name);
				viewer->addLine(roi->points[1], roi->points[2], r, g, b, _name);
				sprintf(_name, "%s_03", name);
				viewer->addLine(roi->points[2], roi->points[3], r, g, b, _name);
				sprintf(_name, "%s_04", name);
				viewer->addLine(roi->points[3], roi->points[0], r, g, b, _name);

				sprintf(_name, "%s_05", name);
				viewer->addLine(roi->points[4], roi->points[5], r, g, b, _name);
				sprintf(_name, "%s_06", name);
				viewer->addLine(roi->points[5], roi->points[6], r, g, b, _name);
				sprintf(_name, "%s_07", name);
				viewer->addLine(roi->points[6], roi->points[7], r, g, b, _name);
				sprintf(_name, "%s_08", name);
				viewer->addLine(roi->points[7], roi->points[4], r, g, b, _name);

				sprintf(_name, "%s_09", name);
				viewer->addLine(roi->points[0], roi->points[4], r, g, b, _name);
				sprintf(_name, "%s_10", name);
				viewer->addLine(roi->points[1], roi->points[5], r, g, b, _name);
				sprintf(_name, "%s_11", name);
				viewer->addLine(roi->points[2], roi->points[6], r, g, b, _name);
				sprintf(_name, "%s_12", name);
				viewer->addLine(roi->points[3], roi->points[7], r, g, b, _name);
			}
		}

		void AddRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::StROI roi, float r, float g, float b, const const char* name)
		{
			SHI::PointCloudPtr pc(new SHI::PointCloud);
			pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.minY, roi.roi.minZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.maxY, roi.roi.minZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.maxY, roi.roi.minZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.minY, roi.roi.minZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.minY, roi.roi.maxZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.maxY, roi.roi.maxZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.maxY, roi.roi.maxZ));
			pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.minY, roi.roi.maxZ));
			AddRoi(viewer, pc, r, g, b, name);
		}

		void AddPartRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::StROI roi, const SHI::CraneAttitudePtr& attitude, int idxPart, float r, float g, float b, const const char* name)
		{
			if (viewer && attitude)
			{
				SHI::PointCloudPtr pc(new SHI::PointCloud);
				pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.minY, roi.roi.minZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.maxY, roi.roi.minZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.maxY, roi.roi.minZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.minY, roi.roi.minZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.minY, roi.roi.maxZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.minX, roi.roi.maxY, roi.roi.maxZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.maxY, roi.roi.maxZ));
				pc->points.push_back(pcl::PointXYZ(roi.roi.maxX, roi.roi.minY, roi.roi.maxZ));

				SHI::Transform::TransformPointCloud(pc, *pc, *attitude, idxPart);

				AddRoi(viewer, pc, r, g, b, name);
			}
		}

		void AddTriRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::StTriangularROI roi, const SHI::CraneAttitudePtr& attitude, float r, float g, float b, const const char* name)
		{
			if (viewer && attitude)
			{				
				float angle = attitude->pose[roi.idxPart];
				float posJibY = roi.length*cos(angle*3.141592 / 180.0);
				float posJibZ = roi.length*sin(angle*3.141592 / 180.0);

				pcl::PointXYZ points[] = { 
					pcl::PointXYZ(roi.roi.minX, posJibY, posJibZ),
					pcl::PointXYZ(roi.roi.minX, roi.roi.minY, roi.roi.minZ),
					pcl::PointXYZ(roi.roi.minX, roi.roi.maxY, roi.roi.maxZ),
					pcl::PointXYZ(roi.roi.maxX, posJibY, posJibZ),
					pcl::PointXYZ(roi.roi.maxX, roi.roi.minY, roi.roi.minZ),
					pcl::PointXYZ(roi.roi.maxX, roi.roi.maxY, roi.roi.maxZ) };

				char _name[256] = "";
				sprintf(_name, "%s_01", name);
				viewer->addLine(points[0], points[1], r, g, b, _name);
				sprintf(_name, "%s_02", name);
				viewer->addLine(points[1], points[2], r, g, b, _name);
				sprintf(_name, "%s_03", name);
				viewer->addLine(points[2], points[0], r, g, b, _name);

				sprintf(_name, "%s_04", name);
				viewer->addLine(points[3], points[4], r, g, b, _name);
				sprintf(_name, "%s_05", name);
				viewer->addLine(points[4], points[5], r, g, b, _name);
				sprintf(_name, "%s_06", name);
				viewer->addLine(points[5], points[3], r, g, b, _name);

				sprintf(_name, "%s_07", name);
				viewer->addLine(points[0], points[3], r, g, b, _name);
				sprintf(_name, "%s_08", name);
				viewer->addLine(points[1], points[4], r, g, b, _name);
				sprintf(_name, "%s_09", name);
				viewer->addLine(points[2], points[5], r, g, b, _name);
			}
		}

		void RemoveRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const char* name)
		{
			char _name[256] = "";
			sprintf(_name, "%s_01", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_02", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_03", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_04", name);
			viewer->removeShape(_name);

			sprintf(_name, "%s_05", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_06", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_07", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_08", name);
			viewer->removeShape(_name);

			sprintf(_name, "%s_09", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_10", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_11", name);
			viewer->removeShape(_name);
			sprintf(_name, "%s_12", name);
			viewer->removeShape(_name);
		}

		void InitializeViewer(pcl::visualization::PCLVisualizer::Ptr viewer)
		{
			viewer->setCameraPosition(100, 0, 100, 0, 0, 1);
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			viewer->addPointCloud(g_emptyPoint, "pointCloud");
			viewer->addPointCloud(g_emptyPointRgb, "pointCluster");
			viewer->addPointCloud(g_emptyPointRgb, "pointCrane");
			viewer->addPointCloud(g_emptyPointRgb, "pointGround");
			viewer->addPointCloud(g_emptyPointRgb, "pointHook");
			viewer->addPointCloud(g_emptyPoint, "pointException");
			viewer->addPointCloud(g_emptyPoint, "pointModel");
			viewer->addPointCloud(g_emptyPoint, "pointCloudBold");
			
			// add model
			for (unsigned int i = 0; i < 5; i++)
			{
				pcl::PolygonMesh p;
				char name[256] = "";
				sprintf(name, "polygonModel%d", i);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				cloud->push_back(pcl::PointXYZ());
				cloud->push_back(pcl::PointXYZ());
				cloud->push_back(pcl::PointXYZ());
				pcl::Vertices v;
				v.vertices.push_back(0);
				v.vertices.push_back(1);
				v.vertices.push_back(2);
				std::vector<pcl::Vertices> vv;
				vv.push_back(v);
				viewer->addPolygonMesh<pcl::PointXYZ>(cloud, vv, name);
			}
		}

		void UpdatePointCloud(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud/*=0*/)
		{
			if (pointCloud != NULL)
			{
				viewer->updatePointCloud(pointCloud, "pointCloud");
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointCloud");
			}
		}

		void UpdateCluster(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud/*=0*/, const SHI::PCLIndicesVectorPtr &indices/*=0*/, const std::vector<unsigned char>* labels/*= 0*/)
		{
			unsigned char targetLabel = SHI::LABEL_OBJECT;

			// exceptions
			if (pointCloud != NULL && indices != NULL && indices != NULL && indices != NULL)
			{
				if (labels->size() == indices->size())
				{
					// generate pointcloud xyzrgb
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for (unsigned int iCluster = 0; iCluster < indices->size(); iCluster++)
					{
						// check label
						if (labels->at(iCluster) == targetLabel)
						{
							for (unsigned int iPoint = 0; iPoint < indices->at(iCluster).indices.size(); iPoint++)
							{
								unsigned int idx = indices->at(iCluster).indices.at(iPoint);

								// xyz to xyzrgb
								pcl::PointXYZRGB pointrgb;
								pointrgb.x = pointCloud->points[idx].x;
								pointrgb.y = pointCloud->points[idx].y;
								pointrgb.z = pointCloud->points[idx].z;
								pointrgb.r = ColorTable::GetColorTableR(iCluster) * 255;
								pointrgb.g = ColorTable::GetColorTableG(iCluster) * 255;
								pointrgb.b = ColorTable::GetColorTableB(iCluster) * 255;

								// push xyzrgb
								cloud->push_back(pointrgb);
							}
						}
					}

					// update pointCluster
					viewer->updatePointCloud(cloud, "pointCluster");
				}
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointCluster");
			}
		}

		void UpdateDistance(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::DistanceInfoVectorPtr &distance /*= 0*/, const std::vector<unsigned char>* labels /*= 0*/, float m_maxDistance /*= 0*/, int numDistance /*= 0*/)
		{
			// clear lines
			for (unsigned int i=0; i<255; i++)
			{
				char name[255] = "";
				sprintf(name, "line%d", i);
				viewer->removeShape(name);

				char name2[255] = "";
				sprintf(name2, "distance%d", i);
				viewer->removeShape(name2);
			}

			// add new ines
			if (distance != NULL && labels != NULL)
			{
				double r = 255, g = 255, b = 255;
				unsigned int count = 0;
				for (unsigned int iDistance=0; iDistance<distance->size(); iDistance++)
				{
					// draw only lesser then m_maxDistance
					if (distance->at(iDistance).Distance > m_maxDistance) continue;

					// draw count lesser then numDistance
					if (count>=numDistance) continue;

					// draw only 'DISTANCE_NORMAL' labeled 
					if (labels->at(iDistance) != SHI::DISTANCE_NORMAL &&
						labels->at(iDistance) != SHI::DISTANCE_HOOK) continue;

					// position to draw text
					float *_posCrane = distance->at(iDistance).PosCrane.xyz;
					float *_posCluster = distance->at(iDistance).PosCluster.xyz;
					pcl::PointXYZ posCrane(_posCrane[0], _posCrane[1], _posCrane[2]);
					pcl::PointXYZ posCluster(_posCluster[0], _posCluster[1], _posCluster[2]);
					pcl::PointXYZ posCenter((posCrane.x + posCluster.x)*0.5, (posCrane.y + posCluster.y)*0.5, (posCrane.z + posCluster.z)*0.5);

					// draw line
					char name[256] = "";
					sprintf(name, "line%d", iDistance);
					viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(posCrane, posCluster, r, g, b, name);

					// draw text
					char name2[256] = "";
					char text[256] = "";
					sprintf(name2, "distance%d", iDistance);
					sprintf(text, "%.2f(%d)", distance->at(iDistance).Distance, iDistance);
					viewer->addText3D(text, posCenter, 1.0, r, g, b, name2);
					count++;
				}
			}
		}

		void UpdateCranePoints(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud /*= 0*/, const SHI::PCLIndicesVectorPtr &indices /*= 0*/, const std::vector<unsigned char>* labels /*= 0*/)
		{
			unsigned char targetLabel = SHI::LABEL_BODY;

			// exceptions
			if (pointCloud != NULL && indices != NULL && indices != NULL && indices != NULL)
			{
				if (labels->size() == indices->size())
				{
					// generate pointcloud xyzrgb
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for (unsigned int iCluster = 0; iCluster < indices->size(); iCluster++)
					{
						// check label
						if (labels->at(iCluster) == targetLabel)
						{
							for (unsigned int iPoint = 0; iPoint < indices->at(iCluster).indices.size(); iPoint++)
							{
								unsigned int idx = indices->at(iCluster).indices.at(iPoint);

								// xyz to xyzrgb
								pcl::PointXYZRGB pointrgb;
								pointrgb.x = pointCloud->points[idx].x;
								pointrgb.y = pointCloud->points[idx].y;
								pointrgb.z = pointCloud->points[idx].z;
								pointrgb.r = 255;
								pointrgb.g = 0;
								pointrgb.b = 0;

								// push xyzrgb
								cloud->push_back(pointrgb);
							}
						}
					}

					// update pointCluster
					viewer->updatePointCloud(cloud, "pointCrane");
				}
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointCrane");
			}
		}

		void UpdateGroundPoints(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud /*= 0*/, const SHI::PCLIndicesVectorPtr &indices /*= 0*/, const std::vector<unsigned char>* labels /*= 0*/)
		{
			unsigned char targetLabel = SHI::LABEL_GROUND;

			// exceptions
			if (pointCloud != NULL && indices != NULL && indices != NULL && indices != NULL)
			{
				if (labels->size() == indices->size())
				{
					// generate pointcloud xyzrgb
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for (unsigned int iCluster = 0; iCluster < indices->size(); iCluster++)
					{
						// check label
						if (labels->at(iCluster) == targetLabel)
						{
							for (unsigned int iPoint = 0; iPoint < indices->at(iCluster).indices.size(); iPoint++)
							{
								unsigned int idx = indices->at(iCluster).indices.at(iPoint);

								// xyz to xyzrgb
								pcl::PointXYZRGB pointrgb;
								pointrgb.x = pointCloud->points[idx].x;
								pointrgb.y = pointCloud->points[idx].y;
								pointrgb.z = pointCloud->points[idx].z;
								pointrgb.r = 100;
								pointrgb.g = 100;
								pointrgb.b = 100;

								// push xyzrgb
								cloud->push_back(pointrgb);
							}
						}
					}

					// update pointCluster
					viewer->updatePointCloud(cloud, "pointGround");
				}
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointGround");
			}
		}

		void UpdateHookPoints(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud /*= 0*/, const SHI::PCLIndicesVectorPtr &indices /*= 0*/, const std::vector<unsigned char>* labels /*= 0*/)
		{
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pointHook");

			unsigned char targetLabel = SHI::LABEL_HOOK;

			// exceptions
			if (pointCloud != NULL && indices != NULL && indices != NULL && indices != NULL)
			{
				if (labels->size() == indices->size())
				{
					// generate pointcloud xyzrgb
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for (unsigned int iCluster = 0; iCluster < indices->size(); iCluster++)
					{
						// check label
						if (labels->at(iCluster) == targetLabel)
						{
							for (unsigned int iPoint = 0; iPoint < indices->at(iCluster).indices.size(); iPoint++)
							{
								unsigned int idx = indices->at(iCluster).indices.at(iPoint);

								// xyz to xyzrgb
								pcl::PointXYZRGB pointrgb;
								pointrgb.x = pointCloud->points[idx].x;
								pointrgb.y = pointCloud->points[idx].y;
								pointrgb.z = pointCloud->points[idx].z;
								pointrgb.r = 170;
								pointrgb.g = 0;
								pointrgb.b = 255;

								// push xyzrgb
								cloud->push_back(pointrgb);
							}
						}
					}

					// update pointCluster
					viewer->updatePointCloud(cloud, "pointHook");
				}
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointHook");
			}
		}

		void UpdateExceptionPoints(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud /*= 0*/, const SHI::PCLIndicesVectorPtr &indices /*= 0*/, const std::vector<unsigned char>* labels /*= 0*/)
		{
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pointException");

			unsigned char targetLabel = SHI::LABEL_EXCEPTION;

			// exceptions
			if (pointCloud != NULL && indices != NULL && indices != NULL && indices != NULL)
			{
				if (labels->size() == indices->size())
				{
					// generate pointcloud xyzrgb
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					for (unsigned int iCluster = 0; iCluster < indices->size(); iCluster++)
					{
						// check label
						if (labels->at(iCluster) == targetLabel)
						{
							for (unsigned int iPoint = 0; iPoint < indices->at(iCluster).indices.size(); iPoint++)
							{
								unsigned int idx = indices->at(iCluster).indices.at(iPoint);

								// xyz to xyzrgb
								pcl::PointXYZRGB pointrgb;
								pointrgb.x = pointCloud->points[idx].x;
								pointrgb.y = pointCloud->points[idx].y;
								pointrgb.z = pointCloud->points[idx].z;
								pointrgb.r = 0x67;
								pointrgb.g = 0;
								pointrgb.b = 0;

								// push xyzrgb
								cloud->push_back(pointrgb);
							}
						}
					}

					// update pointCluster
					viewer->updatePointCloud(cloud, "pointException");
				}
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointException");
			}
		}

		void UpdateHookRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::CraneAttitudePtr & attitude /*= 0*/)
		{
			// 후크 화면 초기화
			for (unsigned int i=0; i<5; i++)
			{
				char name[64] = "";

				// 후크 접지점
				sprintf(name, "hookPos%d", i);
				viewer->removeShape(name);

				// 후크 ROI
				sprintf(name, "hookRoi%d", i);
				RemoveRoi(viewer, name);
				sprintf(name, "hookBoundRoi%d", i);
				RemoveRoi(viewer, name);

				// 인양물 ROI
				sprintf(name, "SalvageRoi%d", i);
				RemoveRoi(viewer, name);
			}

			if (attitude)
			{
				for (unsigned int i=0; i<5; i++)
				{
					if (i >= attitude->numHook) break;

					// 후크 접지점
					char name[64] = "";
					sprintf(name, "hookPos%d", i);
					viewer->addSphere(attitude->hookPoint[i].GetPoint(), 0.5, 0xff, 0x7f, 0x00, name);

					// 후크 ROI
					sprintf(name, "hookRoi%d", i);
					AddRoi(viewer, attitude->hookRoi[i], 1.0f, 1.0f, 0, name);
					sprintf(name, "hookBoundRoi%d", i);
					AddRoi(viewer, attitude->hookBoundRoi[i], 1.0f, 0, 0, name);
				}

				for (unsigned int i = 0; i<5; i++)
				{
					if (i >= attitude->salvageInfo.numSalvageRoi) break;

					// 인양물 ROI
					char name[64] = "";
					sprintf(name, "SalvageRoi%d", i);
					AddRoi(viewer, attitude->salvageInfo.salvageRoi[i], 1.0f, 0, 1.0f, name);
				}
			}
		}
		
		void UpdatePointModel(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::CraneAttitudePtr & attitude /*= 0*/, const std::vector<SHI::PointCloudPtr> *vRefPartPoints /*= 0*/)
		{
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 25, 0, "pointModel");

			if (vRefPartPoints != NULL && attitude != NULL)
			{
				if (vRefPartPoints->size() == attitude->numPart)
				{
					SHI::PointCloudPtr cloudModel(new SHI::PointCloud);
					for (unsigned int i = 0; i < vRefPartPoints->size(); i++)
					{
						SHI::PointCloudPtr cloudTmp(new SHI::PointCloud);
						SHI::Transform::TransformPointCloud(vRefPartPoints->at(i), *cloudTmp, attitude->jointInfo[i], attitude->pose[i]);
						*cloudModel += *cloudTmp;
					}

					viewer->updatePointCloud(cloudModel, "pointModel");
				}				
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointModel");
			}
		}

		void UpdatePolygonModel(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::CraneAttitudePtr & attitude /*= 0*/, const SHI::PolygonModelVector* vModels /*= 0*/)
		{
			for (unsigned int i=0; i<20; i++)
			{
				char name[256] = "";
				sprintf(name, "polygonModel%d", i);
				viewer->removePolygonMesh(name);
			}

			if (vModels != NULL && attitude != NULL)
			{
				for (unsigned int i=0; i<vModels->size(); i++)
				{
					char name[256] = "";
					sprintf(name, "polygonModel%d", i);

					// update model
					SHI::PointCloudPtr cloudTmp(new SHI::PointCloud);
					SHI::Transform::TransformPointCloud(vModels->at(i).cloud, *cloudTmp, attitude->jointInfo[vModels->at(i).part], attitude->pose[vModels->at(i).part]);

					viewer->addPolygonMesh<pcl::PointXYZ>(cloudTmp, vModels->at(i).mesh->polygons, name);
				}
			}
		}

		void UpdateCraneROI(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::CraneAttitudePtr & attitude /*= 0*/)
		{
			// 화면 초기화
			for (unsigned int i = 0; i < 5; i++)
			{
				char name[64] = "";
				sprintf(name, "craneRoi%d", i);
				RemoveRoi(viewer, name);
			}

			if (attitude)
			{
				for (unsigned int i = 0; i < attitude->numPart; i++)
				{
					attitude->craneRoi[0].roi;
					SHI::PointCloudPtr roiPoints(new SHI::PointCloud);
					SHI::PointCloudPtr roiPointsTrans(new SHI::PointCloud);
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.minX, attitude->craneRoi[i].roi.minY, attitude->craneRoi[i].roi.minZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.minX, attitude->craneRoi[i].roi.maxY, attitude->craneRoi[i].roi.minZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.maxX, attitude->craneRoi[i].roi.maxY, attitude->craneRoi[i].roi.minZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.maxX, attitude->craneRoi[i].roi.minY, attitude->craneRoi[i].roi.minZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.minX, attitude->craneRoi[i].roi.minY, attitude->craneRoi[i].roi.maxZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.minX, attitude->craneRoi[i].roi.maxY, attitude->craneRoi[i].roi.maxZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.maxX, attitude->craneRoi[i].roi.maxY, attitude->craneRoi[i].roi.maxZ));
					roiPoints->points.push_back(pcl::PointXYZ(attitude->craneRoi[i].roi.maxX, attitude->craneRoi[i].roi.minY, attitude->craneRoi[i].roi.maxZ));
					SHI::Transform::TransformPointCloud(roiPoints, *roiPointsTrans, *attitude, i);

					char name[64] = "";
					sprintf(name, "craneRoi%d", i);
					AddRoi(viewer, roiPointsTrans, 1.0, 1.0, 1.0, name);
				}
			}
		}

		void UpdateRemoveRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::ClusterParamPtr & param /*= 0*/, const SHI::CraneAttitudePtr& attitude /*= 0*/)
		{
			if (viewer)
			{
				// ROI 리셋
				for (unsigned int i = 0; i < 10; i++)
				{
					char name[64] = "";
					// roi
					sprintf(name, "removeRoi%d", i);
					RemoveRoi(viewer, name);

					// part roi
					for (unsigned int iPart = 0; iPart < 5; iPart++)
					{
						sprintf(name, "removePartRoi%d_%d", iPart, i);
						RemoveRoi(viewer, name);
					}

					// triangle roi
					sprintf(name, "removetriRoi%d", i);
					RemoveRoi(viewer, name);
				}

				if (param && attitude)
				{
					// ROI 그리기
					if (param)
					{
						// roi
						for (unsigned int i = 0; i < param->roiException->vRoi.size(); i++)
						{
							char name[64] = "";
							sprintf(name, "removeRoi%d", i);
							AddRoi(viewer, param->roiException->vRoi[i], 1.0, 1.0, 0, name);

						}
						// part roi
						for (unsigned int iPart = 0; iPart < 5; iPart++)
						{
							for (unsigned int i = 0; i < param->roiException->part[iPart].size(); i++)
							{
								char name[64] = "";
								sprintf(name, "removePartRoi%d_%d", iPart, i);
								AddPartRoi(viewer, param->roiException->part[iPart][i], attitude, iPart, 1.0, 1.0, 0, name);
							}
						}
						// triangle roi
						for (unsigned int i = 0; i < param->roiException->triangular.size(); i++)
						{
							char name[64] = "";
							sprintf(name, "removetriRoi%d", i);
							AddTriRoi(viewer, param->roiException->triangular[i], attitude, 1.0, 1.0, 0, name);

						}
					}
				}
			}
		}

		void UpdateRemoveDistanceRoi(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::DistanceParamPtr & param /*= 0*/, const SHI::CraneAttitudePtr& attitude /*= 0*/)
		{
			if (viewer)
			{
				// ROI 리셋
				for (unsigned int i = 0; i < 10; i++)
				{
					char name[64] = "";
					// roi
					sprintf(name, "removeDistanceRoi%d", i);
					RemoveRoi(viewer, name);

					// triangle roi
					sprintf(name, "removeDistancetriRoi%d", i);
					RemoveRoi(viewer, name);

					// part roi
					for (unsigned int part = 0; part<5; part++)
					{
						sprintf(name, "removeDistancePartRoi%d_%d", part, i);
						RemoveRoi(viewer, name);
					}
				}

				if (param && attitude)
				{
					// ROI 그리기
					if (param)
					{
						// roi
						for (unsigned int i = 0; i < param->roiException->vRoi.size(); i++)
						{
							char name[64] = "";
							sprintf(name, "removeDistanceRoi%d", i);
							AddRoi(viewer, param->roiException->vRoi[i], 1.0, 1.0, 0, name);

						}
						// triangle roi
						for (unsigned int i = 0; i < param->roiException->triangular.size(); i++)
						{
							char name[64] = "";
							sprintf(name, "removeDistancetriRoi%d", i);
							AddTriRoi(viewer, param->roiException->triangular[i], attitude, 1.0, 1.0, 0, name);

						}
						// part roi
						for (unsigned int part = 0; part < 5; part++)
						{
							for (unsigned int i = 0; i < param->roiException->part[part].size(); i++)
							{
								char name[64] = "";
								sprintf(name, "removeDistancePartRoi%d_%d", part, i);
								AddPartRoi(viewer, param->roiException->part[part][i], attitude, part, 1.0, 1.0, 0, name);
							}
						}
					}
				}
			}
		}

		void UpdateRemoveDistancePartRoi(pcl::visualization::PCLVisualizer::Ptr viewer, int part, const SHI::DistanceParamPtr & param /*= 0*/, const SHI::CraneAttitudePtr& attitude /*= 0*/)
		{
			if (viewer)
			{
				// ROI 리셋
				for (unsigned int i = 0; i < 10; i++)
				{
					char name[64] = "";
					sprintf(name, "removeDistancePartOnlyRoi%d_%d", part, i);
					RemoveRoi(viewer, name);
				}

				if (param && attitude)
				{
					// ROI 그리기
					if (param)
					{
						// part roi
						for (unsigned int i = 0; i < param->roiException->partOnly[part].size(); i++)
						{
							char name[64] = "";
							sprintf(name, "removeDistancePartOnlyRoi%d_%d", part, i);
							AddPartRoi(viewer, param->roiException->partOnly[part][i], attitude, part, 1.0, 0, 1.0, name);
						}
					}
				}
			}
		}

		void UpdatePointCloudBold(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::PointCloudPtr &pointCloud /*= 0*/)
		{
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 255, "pointCloudBold");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pointCloudBold");
			if (pointCloud != NULL)
			{
				viewer->updatePointCloud(pointCloud, "pointCloudBold");
			}
			else
			{
				viewer->updatePointCloud(g_emptyPoint, "pointCloudBold");
			}
		}
		
		void UpdateDistanceBold(pcl::visualization::PCLVisualizer::Ptr viewer, const SHI::DistanceInfoVectorPtr &distance /*= 0*/, const std::vector<unsigned int> *indices/*= 0*/)
		{
			// clear lines
			for (unsigned int i = 0; i < 255; i++)
			{
				char name[255] = "";
				sprintf(name, "lineBold%d", i);
				viewer->removeShape(name);

				char name2[255] = "";
				sprintf(name2, "distanceBold%d", i);
				viewer->removeShape(name2);
			}

			// add new ines
			if (distance != NULL)
			{
				double r = 0x72 / 255.0, g = 0x00 / 255.0, b = 0xDA / 255.0;
				for (unsigned int iDistance = 0; iDistance < distance->size(); iDistance++)
				{
					// check if variable 'iDistance' is a member of indices
					bool bMember = false;
					for (unsigned int i=0; i<indices->size() && bMember == false; i++)
					{
						if (indices->at(i) == iDistance)
						{
							bMember = true;
						}
					}

					// draw bold distance
					if (bMember)
					{
						float *_posCrane = distance->at(iDistance).PosCrane.xyz;
						float *_posCluster = distance->at(iDistance).PosCluster.xyz;
						pcl::PointXYZ posCrane(_posCrane[0], _posCrane[1], _posCrane[2]);
						pcl::PointXYZ posCluster(_posCluster[0], _posCluster[1], _posCluster[2]);
						pcl::PointXYZ posCenter((posCrane.x + posCluster.x)*0.5, (posCrane.y + posCluster.y)*0.5, (posCrane.z + posCluster.z)*0.5);

						char name[256] = "";
						sprintf(name, "lineBold%d", iDistance);
						viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(posCrane, posCluster, r, g, b, name);

						char text[256] = "";
						char name2[256] = "";
						sprintf(name2, "distanceBold%d", iDistance);
						sprintf(text, "%.2f(%d)", distance->at(iDistance).Distance, iDistance);
						viewer->addText3D(text, posCenter, 1.0, r, g, b, name2);

						viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, name);
					}
				}
			}
		}

	}
}