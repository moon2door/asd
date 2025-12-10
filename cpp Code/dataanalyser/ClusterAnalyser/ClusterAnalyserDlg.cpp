
// ClusterAnalyserDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "ClusterAnalyser.h"
#include "ClusterAnalyserDlg.h"
#include "afxdialogex.h"
//#pragma comment(linker, "/entry:WinMainCRTStartup /subsystem:console")

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// CClusterAnalyserDlg 대화 상자
#include <Utility/Types.h>
#include <Utility/DrawUtility.h>
#include <Utility/Convert.h>
#include <Utility/FileUtility.h>
#include <Utility/LoadCraneModel.h>
#include <Utility/filter.h>
#include <Config/ClusteringParameter.h>
#include <Config/InitialAttitude.h>


#include <Routine/Include/Base/RoutineUtility.h>

// Message define
#define WM_DATA_ANALYZER_UPDATE_GUI (WM_USER+1001)

// MessageMap
#define ON_WM_DATA_ANALYZER_UPDATE_GUI(memberFxn) \
{ (WM_DATA_ANALYZER_UPDATE_GUI), 0, 0, 0, AfxSig_v_v_v, (AFX_PMSG)(AFX_PMSGW) (static_cast< void (AFX_MSG_CALL CWnd::*)(void) > (memberFxn)) },

bool FindClusterFiles(std::vector<std::string> &files, const char* path, const char* extension)
{
	bool ret = false;
	char _path[256] = "";
	sprintf(_path, "%s\\*.%s", path, extension);

	WIN32_FIND_DATA findData;
	const HANDLE hFind = FindFirstFile(_path, &findData);
	if (hFind != INVALID_HANDLE_VALUE)
	{
		do
		{
			ret = true;
			std::string str = std::string(path) + "\\" + std::string(findData.cFileName);
			files.push_back(str);
		} while (FindNextFile(hFind, &findData));
	}
	FindClose(hFind);
	return ret;
}

CClusterAnalyserDlg::CClusterAnalyserDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_CLUSTERANALYSER_DIALOG, pParent)
	, m_id(10) 
	, m_pierId(7)
	, m_craneId(0)
	, m_nPoints(0)
	, m_nClusters(0)
	, m_nLabels(0)
	, m_bWaiting(false)
	, m_currentData(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

}

//pjh
CClusterAnalyserDlg::~CClusterAnalyserDlg() 
{
	m_threadDraw.StopThread();
	m_viewer = nullptr;
}
//

void CClusterAnalyserDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHECK_DrawPoints, m_checkDrawPoints);
	DDX_Control(pDX, IDC_CHECK_DrawClusters, m_checkDrawClusters);
	DDX_Control(pDX, IDC_CHECK_DrawCrane, m_checkDrawCrane);
	DDX_Control(pDX, IDC_CHECK_DrawGround, m_checkDrawGround);
	DDX_Control(pDX, IDC_CHECK_DrawHook, m_checkDrawHook);
	DDX_Control(pDX, IDC_CHECK_HookRoi, m_hookRoi);
	DDX_Control(pDX, IDC_CHECK_PointModel, m_PointModel);
	DDX_Control(pDX, IDC_CHECK_CraneRoi, m_craneRoi);
	DDX_Control(pDX, IDC_CHECK_RemoveRoi, m_removeRoi);
	DDX_Control(pDX, IDC_CHECK_Show3DWindow, m_checkShow3DWindow);
	DDX_Text(pDX, IDC_EDIT_ID, m_id);
	DDX_Text(pDX, IDC_EDIT_PierId, m_pierId);
	DDX_Text(pDX, IDC_EDIT_CraneId, m_craneId);
	DDX_Text(pDX, IDC_EDIT_nPoints, m_nPoints);
	DDX_Text(pDX, IDC_EDIT_nCluster, m_nClusters);
	DDX_Text(pDX, IDC_EDIT_nLabels, m_nLabels);
	DDX_Control(pDX, IDC_LIST_Clusters, m_listCluster);
	DDX_Text(pDX, IDC_TXT_CurrentData, m_currentData);
	DDX_Control(pDX, IDC_CHECK_DrawException, m_checkDrawException);
}

void CClusterAnalyserDlg::OnCluster(SHI::Data::StCluster* pData)
{
	if (m_bWaiting)
	{
		
		UpdateCluster(pData);
	}
}

void CClusterAnalyserDlg::UpdateCluster(SHI::Data::StCluster* pData)
{
	const boost::shared_ptr<SHI::Data::StCluster> data(new SHI::Data::StCluster);
	memcpy(data.get(), pData, sizeof(SHI::Data::StCluster));
	m_cluster = data;
	
	// update mfc gui
	PostMessage(WM_DATA_ANALYZER_UPDATE_GUI, 0, 0);
	
	// update pcl viewer
	RedrawViewer();
}

void CClusterAnalyserDlg::RedrawViewer()
{
	// ReSharper disable once CppExpressionWithoutSideEffects
	m_eUpdateViewer.SetEvent();
}

typedef std::pair<pcl::visualization::PCLVisualizer::Ptr, boost::shared_ptr<SHI::Data::StCluster>> MYHANDLE;

void OnPointPicking(const pcl::visualization::PointPickingEvent& event, void* param)
{
	auto handle = static_cast<MYHANDLE*>(param);
	//m_viewer = handle->first;
	const pcl::visualization::PCLVisualizer::Ptr viewer = handle->first;
	const boost::shared_ptr<SHI::Data::StCluster> data = handle->second;

	// picked point
	float_t x = 0, y = 0, z = 0;
	event.getPoint(x, y, z);
	const SHI::Point3D posPicked(x, y, z);
	const SHI::PointCloudPtr cloudPicked(new SHI::PointCloud);	
	cloudPicked->push_back(posPicked);

	// extract xyz, indices, labels from data
	const SHI::PCLIndicesVectorPtr indices(new SHI::PCLIndicesVector);
	const SHI::PointCloudPtr cloud(new SHI::PointCloud);
	std::vector<unsigned char> labels;
	SHI::Convert::Cluster2XYZIndices(data.get(), *cloud, *indices, labels);

	// extract attitude from data
	const SHI::CraneAttitudePtr attitude(new SHI::CraneAttitude);
	*attitude = *reinterpret_cast<SHI::CraneAttitude*>(&data->attitude);

	// find including cluster id
	bool bContains = false;
	uint32_t clusterId = 0;
	for (uint32_t iCluster = 0; !bContains && iCluster<indices->size(); iCluster++)
	{
		for (uint32_t i = 0; i < indices->at(iCluster).indices.size(); i++)
		{
			uint32_t idx = indices->at(iCluster).indices[i];
			SHI::StROI roi = SHI::StROI(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5) + cloud->at(idx);
			if (roi.IsInlier(posPicked))
			{
				bContains = true;
				clusterId = iCluster;
				break;
			}
		}
	}
	
	// draw picked contents to viewer
	const std::string idRoi = "pickedRoi";
	const std::string idPoint = "pickedPoint";
	const std::string idText = "pickedPointText";
	char str[256] = "";

	if (bContains)
	{
		// cluster points
		const SHI::PointCloudPtr cloudCluster(new SHI::PointCloud);
		SHI::Filter::FilterIndices(cloud, *cloudCluster, indices->at(clusterId).indices);

		// cluster ROI
		const SHI::StROI roiCluster(cloudCluster);
		printf("%d (%f, %f, %f) \n", clusterId, x, y, z);
		// ReSharper disable once CppDeprecatedEntity
		sprintf(str, "point : (%.3f, %.3f, %.3f)\ncluster : %d\ncluster roi : (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)", 
		        x, y, z, clusterId, roiCluster.roi.minX, roiCluster.roi.maxX, roiCluster.roi.minY, roiCluster.roi.maxY, roiCluster.roi.minZ, roiCluster.roi.maxZ);

		// add roi to viewer
		SHI::Utility::RemoveRoi(viewer, idRoi.c_str());
		SHI::Utility::AddRoi(viewer, roiCluster, 1.0, 1.0, 1.0, idRoi.c_str());
	}
	else
	{
		SHI::Utility::RemoveRoi(viewer, idRoi.c_str());
		// ReSharper disable once CppDeprecatedEntity
		sprintf(str, "point : (%.3f, %.3f, %.3f)", x, y, z);
	}

	// cluster ROI
	viewer->removePointCloud("pickedPoint");
	viewer->addPointCloud<SHI::Point3D>(cloudPicked, idPoint);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1.0, idPoint);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, idPoint);
	viewer->removeShape(idText);
	viewer->addText3D(str, posPicked, 1.0, 1.0, 1.0, 1.0, idText);
}

void OnKeyboardInput(const pcl::visualization::KeyboardEvent& event, void* param)
{
	auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(param);
	
	switch (event.getKeyCode())
	{
	case '8':
		pViewer->setCameraPosition(0, 100, 0, 0, 0, 1);
		break;
	case '2':
		pViewer->setCameraPosition(100, -100, 0, 0, 0, 1);
		break;
	case '6':
		pViewer->setCameraPosition(100, 0, 0, 0, 0, 1);
		break;
	case '4':
		pViewer->setCameraPosition(-100, 0, 0, 0, 0, 1);
		break;
	case '5':
		pViewer->setCameraPosition(0, 0, 100, 0, 1, 0);
		break;
	default: 
		break;
	}
}

void CClusterAnalyserDlg::ThreadDraw()
{
	boost::shared_ptr<SHI::Data::StCluster> data;

	m_viewer.reset(new pcl::visualization::PCLVisualizer("cluster viewer"));
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cluster viewer"));
	SHI::Utility::InitializeViewer(m_viewer);

	// register picking callback
	MYHANDLE handle(m_viewer, data);
	m_viewer->registerPointPickingCallback(OnPointPicking, &handle);
	m_viewer->registerKeyboardCallback(OnKeyboardInput, m_viewer.get());
	
	// load crane point model
	//std::string szPier = 7; Routine::GetConfigString("CraneType", "Pier", "G4", "CraneType.json");
	int32_t curPier = m_pierId;// SHI::ConvPierInt(szPier);
	int32_t curCraneId = m_craneId;// Routine::GetConfigInt("CraneType", "Type", 0, "CraneType.json");
	//SHI::CraneAttitudePtr attitude(new SHI::CraneAttitude);//pjh
	SHI::CraneAttitude initialAttitude;
	SHI::ClusterParamPtr clusterParam(new SHI::ClusterParam);
	std::vector<SHI::PointCloudPtr> vPointModels;
	SHI::GetClusterParam(curPier, curCraneId, *clusterParam);
	SHI::GetInitialAttitude(curPier, curCraneId, initialAttitude);
	//pjh
	//SHI::LoadPointModel(vPointModels, *attitude, clusterParam->fnameModel);
	SHI::LoadPointModel(vPointModels, initialAttitude, clusterParam->fnameModel);
	//

	while (m_threadDraw.IsRunThread() && m_viewer->wasStopped() == false)
	{
		if (m_eUpdateViewer.WaitForEvent(10))
		{
			// get cluster data
			data = m_cluster;

			// update picking handle
			handle = MYHANDLE(m_viewer, data);

			// process draw
			if (data)
			{
				// extract xyz, indices, labels from data
				SHI::PCLIndicesVectorPtr indices(new SHI::PCLIndicesVector);
				SHI::PointCloudPtr cloud(new SHI::PointCloud);
				std::vector<unsigned char> labels;
				SHI::Convert::Cluster2XYZIndices(data.get(), *cloud, *indices, labels);

				printf("cx = %.1f, cy = %.1f, cz = %.1f, tx = %.1f, ty = %.1f, tz = %.1f, rx = %.1f, ry = %.1f, rz = %.1f \n"
					, data->attitude.jointInfo[18]
					, data->attitude.jointInfo[19]
					, data->attitude.jointInfo[20]
					, data->attitude.jointInfo[21]
					, data->attitude.jointInfo[22]
					, data->attitude.jointInfo[23]
					, data->attitude.jointInfo[24]
					, data->attitude.jointInfo[25]
					, data->attitude.jointInfo[26]
				);
				// extract attitude from data
				//pjhSHI::CraneAttitude *attitude = *((SHI::CraneAttitude*)&data->attitude);
				SHI::CraneAttitudePtr attitude(new SHI::CraneAttitude);
				*attitude = *((SHI::CraneAttitude*)&data->attitude);

				// update gui data
				if (m_checkDrawPoints.GetCheck())
				{
					SHI::Utility::UpdatePointCloud(m_viewer, cloud);
				}
				else
				{
					SHI::Utility::UpdatePointCloud(m_viewer);
				}

				if (m_checkDrawClusters.GetCheck())
				{
					SHI::Utility::UpdateCluster(m_viewer, cloud, indices, &labels);
				}
				else
				{
					SHI::Utility::UpdateCluster(m_viewer);
				}

				if (m_checkDrawCrane.GetCheck())
				{
					SHI::Utility::UpdateCranePoints(m_viewer, cloud, indices, &labels);
				}
				else
				{
					SHI::Utility::UpdateCranePoints(m_viewer);
				}

				if (m_checkDrawGround.GetCheck())
				{
					SHI::Utility::UpdateGroundPoints(m_viewer, cloud, indices, &labels);
				}
				else
				{
					SHI::Utility::UpdateGroundPoints(m_viewer);
				}

				if (m_checkDrawHook.GetCheck())
				{
					SHI::Utility::UpdateHookPoints(m_viewer, cloud, indices, &labels);
				}
				else
				{
					SHI::Utility::UpdateHookPoints(m_viewer);
				}

				if (m_checkDrawException.GetCheck())
				{
					SHI::Utility::UpdateExceptionPoints(m_viewer, cloud, indices, &labels);
				}
				else
				{
					SHI::Utility::UpdateExceptionPoints(m_viewer);
				}				

				if (m_hookRoi.GetCheck())
				{

					SHI::Utility::UpdateHookRoi(m_viewer, attitude);
				}
				else
				{
					SHI::Utility::UpdateHookRoi(m_viewer);
				}

				if (m_PointModel.GetCheck())
				{
					// reload point model
					if (attitude->pierId != curPier || attitude->craneId != curCraneId)
					{
						SHI::GetClusterParam(attitude->pierId, attitude->craneId, *clusterParam);
						SHI::LoadPointModel(vPointModels, *attitude, clusterParam->fnameModel);
					}

					// update point model
					SHI::Utility::UpdatePointModel(m_viewer, attitude, &vPointModels);
				}
				else
				{
					SHI::Utility::UpdatePointModel(m_viewer, 0, 0);
				}

				if (m_craneRoi.GetCheck())
				{
					SHI::Utility::UpdateCraneROI(m_viewer, attitude);
				}
				else
				{
					SHI::Utility::UpdateCraneROI(m_viewer, 0);
				}

				if (m_removeRoi.GetCheck())
				{
					// reload cluster params
					if (attitude->pierId != curPier || attitude->craneId != curCraneId)
					{
						SHI::GetClusterParam(attitude->pierId, attitude->craneId, *clusterParam);
					}
					SHI::Utility::UpdateRemoveRoi(m_viewer, clusterParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveRoi(m_viewer);
				}

				// bold point
				SHI::PointCloudPtr pointsBold(new SHI::PointCloud);
				for (uint32_t i = 0; i < indices->size(); i++)
				{
					if (m_listCluster.GetCheck(i) == TRUE)
					{
						for (uint32_t iPoint = 0; iPoint < indices->at(i).indices.size(); iPoint++)
						{
							uint32_t idx = indices->at(i).indices.at(iPoint);
							pointsBold->push_back(cloud->at(idx));
						}
					}
				}
				SHI::Utility::UpdatePointCloudBold(m_viewer, pointsBold);
			}
		}
		m_viewer->spinOnce();
	}

	m_threadDraw.StopThread();
}

BEGIN_MESSAGE_MAP(CClusterAnalyserDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_DATA_ANALYZER_UPDATE_GUI(OnUpdateGUI)
	ON_BN_CLICKED(IDC_BTN_ExportPcd, &CClusterAnalyserDlg::OnBnClickedBtnExportpcd)
	ON_BN_CLICKED(IDC_BTN_SaveFile, &CClusterAnalyserDlg::OnBnClickedBtnSavefile)
	ON_BN_CLICKED(IDC_BTN_LoadFile, &CClusterAnalyserDlg::OnBnClickedBtnLoadfile)
	ON_BN_CLICKED(IDC_BTN_Prev, &CClusterAnalyserDlg::OnBnClickedBtnPrev)
	ON_BN_CLICKED(IDC_BTN_Next, &CClusterAnalyserDlg::OnBnClickedBtnNext)
	ON_BN_CLICKED(IDC_BTN_WriteMemory, &CClusterAnalyserDlg::OnBnClickedBtnWritememory)
	ON_BN_CLICKED(IDC_BTN_ReadMemory, &CClusterAnalyserDlg::OnBnClickedBtnReadmemory)
	ON_BN_CLICKED(IDC_BTN_WaitSync, &CClusterAnalyserDlg::OnBnClickedBtnWaitsync)
	ON_BN_CLICKED(IDC_CHECK_DrawPoints, &CClusterAnalyserDlg::OnBnClickedCheckDrawpoints)
	ON_BN_CLICKED(IDC_CHECK_DrawClusters, &CClusterAnalyserDlg::OnBnClickedCheckDrawclusters)
	ON_BN_CLICKED(IDC_CHECK_DrawCrane, &CClusterAnalyserDlg::OnBnClickedCheckDrawcrane)
	ON_BN_CLICKED(IDC_CHECK_DrawGround, &CClusterAnalyserDlg::OnBnClickedCheckDrawground)
	ON_BN_CLICKED(IDC_CHECK_DrawHook, &CClusterAnalyserDlg::OnBnClickedCheckDrawhook)
	ON_BN_CLICKED(IDC_CHECK_HookRoi, &CClusterAnalyserDlg::OnBnClickedCheckHookroi)
	ON_BN_CLICKED(IDC_CHECK_PointModel, &CClusterAnalyserDlg::OnBnClickedCheckPointmodel)
	ON_BN_CLICKED(IDC_CHECK_CraneRoi, &CClusterAnalyserDlg::OnBnClickedCheckCraneroi)
	ON_BN_CLICKED(IDC_CHECK_RemoveRoi, &CClusterAnalyserDlg::OnBnClickedCheckRemoveroi)
	ON_BN_CLICKED(IDC_CHECK_Show3DWindow, &CClusterAnalyserDlg::OnBnClickedCheckShow3dwindow)
	ON_NOTIFY(NM_CLICK, IDC_LIST_Clusters, &CClusterAnalyserDlg::OnClickListClusters)
	ON_BN_CLICKED(IDC_CHECK_DrawException, &CClusterAnalyserDlg::OnBnClickedCheckDrawexception)
	ON_WM_DESTROY()//pjh
END_MESSAGE_MAP()

// CClusterAnalyserDlg 메시지 처리기

BOOL CClusterAnalyserDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	//DevLib::Utility::Console::ConsoleOpen("test");
	m_checkDrawClusters.SetCheck(TRUE);
	m_checkDrawCrane.SetCheck(TRUE);
	m_checkDrawGround.SetCheck(TRUE);
	m_checkDrawHook.SetCheck(TRUE);
	m_checkDrawException.SetCheck(TRUE);
	m_eUpdateViewer.Create();


	m_listCluster.SetExtendedStyle(LVS_EX_CHECKBOXES);
	m_listCluster.InsertColumn(0, "index", LVCFMT_CENTER, 50);
	m_listCluster.InsertColumn(1, "cluster num", LVCFMT_CENTER, 100);
	m_listCluster.InsertColumn(2, "label", LVCFMT_CENTER, 100);

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CClusterAnalyserDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int32_t cxIcon = GetSystemMetrics(SM_CXICON);
		int32_t cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int32_t x = (rect.Width() - cxIcon + 1) / 2;
		int32_t y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CClusterAnalyserDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CClusterAnalyserDlg::OnUpdateGUI(void)
{
	boost::shared_ptr<SHI::Data::StCluster> cluster = m_cluster;
	if (cluster)
	{
		UpdateData(TRUE);

		// update ctrls
		m_pierId = cluster->attitude.pierId;
		m_craneId = cluster->attitude.craneId;
		m_nPoints = cluster->GetXYZSize();
		m_nClusters = cluster->GetClusterInfoSize();
		m_nLabels = cluster->GetLabelSize();

		//update list
		m_listCluster.DeleteAllItems();
		if (cluster->GetClusterInfoSize() == cluster->GetLabelSize())
		{
			for (uint32_t i = 0; i < cluster->GetClusterInfoSize(); i++)
			{
				CString itemNameCluster, fieldCluster1, fieldCluster2, fieldCluster3;
				itemNameCluster.Format("item %d", i);
				fieldCluster1.Format("%d", i);
				fieldCluster2.Format("%d", cluster->GetClusterInfo()[i].Size);
				fieldCluster3.Format("%d", cluster->GetLabel()[i]);

				m_listCluster.InsertItem(i, itemNameCluster);
				m_listCluster.SetItemText(i, 0, fieldCluster1);
				m_listCluster.SetItemText(i, 1, fieldCluster2);
				m_listCluster.SetItemText(i, 2, fieldCluster3);

				//if (m_bSelectAllClusters) m_listClusters.SetCheck(i);
			}
		}
		UpdateData(FALSE);
	}
}

void CClusterAnalyserDlg::OnBnClickedBtnExportpcd()
{
	boost::shared_ptr<SHI::Data::StCluster> cluster = m_cluster;
	if (cluster)
	{
		char filter[] = "pcd Files (*.pcd)|*.pcd|";
		CFileDialog fileDlg(FALSE, "pcd file", "*.pcd", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, filter, NULL);
		if (fileDlg.DoModal() == IDOK)
		{
			CString fname = fileDlg.GetPathName();
			
			// cluster to pcd
			SHI::PCLIndicesVector indices;
			SHI::PointCloud cloud;
			std::vector<unsigned char> labels;
			SHI::Convert::Cluster2XYZIndices(cluster.get(), cloud, indices, labels);

			// save pcd
			pcl::io::savePCDFileBinary(std::string((LPCTSTR)fname), cloud);
		}
	}
}

void CClusterAnalyserDlg::OnBnClickedBtnSavefile()
{
	boost::shared_ptr<SHI::Data::StCluster> cluster = m_cluster;
	if (cluster)
	{
		char filter[] = "StCluster Files (*.StCluster)|*.StCluster|";
		CFileDialog fileDlg(FALSE, "StCluster", "*.StCluster", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, filter, NULL);
		if (fileDlg.DoModal() == IDOK)
		{
			// save cluster
			CString fname = fileDlg.GetPathName();
			SHI::FileUtility::SaveClusterFile(*cluster, std::string((LPCTSTR)fname));
		}
	}
}

void CClusterAnalyserDlg::OnBnClickedBtnLoadfile()
{
	boost::shared_ptr<SHI::Data::StCluster> cluster = boost::shared_ptr<SHI::Data::StCluster>(new SHI::Data::StCluster);

	CFileDialog fileDlg(TRUE, NULL, "cluster.StCluster", OFN_HIDEREADONLY | OFN_NOCHANGEDIR, "Cluster Files (*.StCluster)|*.StCluster||");
	if (fileDlg.DoModal() == IDOK)
	{
		CString fname = fileDlg.GetPathName();
		if (SHI::FileUtility::LoadClusterFile(*cluster, std::string((LPCTSTR)fname)))
		{
			// load cluster
			UpdateCluster(cluster.get());

			// update directory info
			m_currentData = fileDlg.GetFileName();
			m_selectedFile = fileDlg.GetPathName();
			m_selectedFolder = fileDlg.GetFolderPath();
		}
	}
	UpdateData(FALSE);
}

void CClusterAnalyserDlg::OnBnClickedBtnPrev()
{
	if (!m_selectedFolder.empty())
	{
		std::vector<std::string> vClusterFiles;
		FindClusterFiles(vClusterFiles, m_selectedFolder.c_str(), "StCluster");

		if (vClusterFiles.size())
		{
			std::string newFile;

			// 현재 선택된 폴더 탐색하여 다음 파일을 찾음
			for (int32_t i = 0; i < vClusterFiles.size(); i++)
			{
				if (vClusterFiles[i].compare(m_selectedFile) == 0 && (i - 1) >= 0)
				{
					newFile = vClusterFiles[i - 1];
					break;
				}
			}

			// 다음 파일이 있으면 해당 데이터 불러와서 적용
			if (!newFile.empty())
			{
				SHI::Data::StCluster *pData = new SHI::Data::StCluster;
				if (SHI::FileUtility::LoadClusterFile(*pData, newFile.c_str()))
				{
					m_selectedFile = newFile;
					m_currentData = newFile.c_str();

					int32_t s = newFile.find_last_of('\\');
					if (s != -1)
					{
						m_currentData = newFile.substr(s + 1, newFile.size()).c_str();
						UpdateCluster(pData);
					}
				}
			}
		}
	}
}

void CClusterAnalyserDlg::OnBnClickedBtnNext()
{
	if (!m_selectedFolder.empty())
	{
		std::vector<std::string> vClusterFiles;
		FindClusterFiles(vClusterFiles, m_selectedFolder.c_str(), "StCluster");

		if (vClusterFiles.size())
		{
			std::string newFile;

			// 현재 선택된 폴더 탐색하여 다음 파일을 찾음
			for (int32_t i = 0; i < vClusterFiles.size(); i++)
			{
				if (vClusterFiles[i].compare(m_selectedFile) == 0 && (i + 1) < vClusterFiles.size())
				{
					newFile = vClusterFiles[i + 1];
					break;
				}
			}

			// 다음 파일이 있으면 해당 데이터 불러와서 적용
			if (!newFile.empty())
			{
				SHI::Data::StCluster *pData = new SHI::Data::StCluster;
				if (SHI::FileUtility::LoadClusterFile(*pData, newFile.c_str()))
				{
					m_selectedFile = newFile;
					m_currentData = newFile.c_str();

					int32_t s = newFile.find_last_of('\\');
					if (s != -1)
					{
						m_currentData = newFile.substr(s + 1, newFile.size()).c_str();
						UpdateCluster(pData);
					}
				}
			}
		}
	}
}

void CClusterAnalyserDlg::OnBnClickedBtnWritememory()
{
	if (m_cluster)
	{
		UpdateData(TRUE);
		SHI::Interface::Cluster::Stub::CStubCluster stub;
		stub.GetStubCluster()->Create(m_id);
		stub.GetStubCluster()->WriteData(m_cluster.get());
	}
}

void CClusterAnalyserDlg::OnBnClickedBtnReadmemory()
{
	UpdateData(TRUE);
	UpdateData(FALSE);
	SHI::Data::StCluster* pData = new SHI::Data::StCluster;
	static CProxyClusterObject obj;
	obj.GetProxyCluster()->Create(m_id);
	if (obj.GetProxyCluster()->ReadBuffer(pData))
	{
		int32_t a1 = pData->GetXYZSize();
		int32_t a2 = pData->GetClusterIndicesSize();
		int32_t a3 = pData->GetClusterInfoSize();
		int32_t a4 = pData->GetLabelSize();

		UpdateCluster(pData); 
	}
	delete pData;
}

void CClusterAnalyserDlg::OnBnClickedBtnWaitsync()
{
	if (m_bWaiting == true)
	{
		m_bWaiting = false;
		SetDlgItemText(IDC_BTN_WaitSync, "Wait Sync");
	}
	else
	{
		m_bWaiting = true;
		UpdateData(TRUE);
		GetProxyCluster()->Create(m_id);
		SetDlgItemText(IDC_BTN_WaitSync, "Stop Sync");
	}	
}

void CClusterAnalyserDlg::OnBnClickedCheckDrawpoints()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckDrawclusters()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckDrawcrane()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckDrawground()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckDrawhook()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckDrawexception()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckHookroi()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckPointmodel()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckCraneroi()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckRemoveroi()
{
	RedrawViewer();
}

void CClusterAnalyserDlg::OnBnClickedCheckShow3dwindow()
{
	if (m_checkShow3DWindow.GetCheck() == true)
	{
		m_threadDraw.StartThread(&CClusterAnalyserDlg::ThreadDraw, this);
		RedrawViewer();
	}
	else
	{
		m_threadDraw.StopThread();
	}
}


void CClusterAnalyserDlg::OnClickListClusters(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if (pNMItemActivate->iItem >= 0)
	{
		RedrawViewer();
	}
	*pResult = 0;
}

//pjh
//BOOL CClusterAnalyserDlg::DestroyWindow()
//{
//
//}
//