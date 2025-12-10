
// DistanceAnalyserDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "DistanceAnalyser.h"
#include "DistanceAnalyserDlg.h"
#include "afxdialogex.h"

// Because new operator was defined in eigen3
//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// CDistanceAnalyserDlg 대화 상자
#include <Utility/Types.h>
#include <Utility/DrawUtility.h>
#include <Utility/Convert.h>
#include <Utility/FileUtility.h>
#include <Utility/LoadCraneModel.h>
#include <Config/DistanceParameter.h>
#include <Config/InitialAttitude.h>
#include <Utility/LoadCraneModel.h>

//#include <pcl/io/pcd_io.h>

// Message define
#define WM_DATA_ANALYZER_UPDATE_GUI (WM_USER+1001)

// MessageMap
#define ON_WM_DATA_ANALYZER_UPDATE_GUI(memberFxn) \
{ (WM_DATA_ANALYZER_UPDATE_GUI), 0, 0, 0, AfxSig_v_v_v, (AFX_PMSG)(AFX_PMSGW) (static_cast< void (AFX_MSG_CALL CWnd::*)(void) > (memberFxn)) },

bool FindDistanceFiles(std::vector<std::string> &files, const char* path, const char* extension)
{
	bool ret = false;
	char _path[256] = "";
	sprintf(_path, "%s\\*.%s", path, extension);

	WIN32_FIND_DATA findData;
	HANDLE hFind;
	hFind = FindFirstFile(_path, &findData);
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

CDistanceAnalyserDlg::CDistanceAnalyserDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_DISTANCEANALYSER_DIALOG, pParent)
	, m_id(10)
	, m_bWaiting(false)
	, m_txtCurrentData(_T(""))
	, m_pierId(7)
	, m_craneId(0)
	, m_nPoints(0)
	, m_nClusters(0)
	, m_nLabels(0)
	, m_nDistance(0)
	, m_distanceLabels(0)
	, m_minDistance(0)
	, m_maxDistance(20.0)
	, m_numDistance(10)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

//pjh
CDistanceAnalyserDlg::~CDistanceAnalyserDlg() 
{

}
//

void CDistanceAnalyserDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_ID, m_id);
	DDX_Text(pDX, IDC_TXT_currentData, m_txtCurrentData);
	DDX_Text(pDX, IDC_EDIT_PierID, m_pierId);
	DDX_Text(pDX, IDC_EDIT_CraneID, m_craneId);
	DDX_Text(pDX, IDC_EDIT_nPoints, m_nPoints);
	DDX_Text(pDX, IDC_EDIT_nClusters, m_nClusters);
	DDX_Text(pDX, IDC_EDIT_nLabels, m_nLabels);
	DDX_Text(pDX, IDC_EDIT_nDistances, m_nDistance);
	DDX_Text(pDX, IDC_EDIT_distanceLabels, m_distanceLabels);
	DDX_Control(pDX, IDC_LIST_Clusters, m_listCluster);
	DDX_Control(pDX, IDC_LIST_Distance, m_listDistance);
	DDX_Control(pDX, IDC_CHECK_DrawPoints, m_checkDrawPoints);
	DDX_Control(pDX, IDC_CHECK_DrawClusters, m_checkDrawClusters);
	DDX_Control(pDX, IDC_CHECK_ExceptionRoi, m_checkDrawExceptionRoi);
	DDX_Control(pDX, IDC_CHECK_Show3dWindow, m_checkShow3DWindow);
	DDX_Control(pDX, IDC_CHECK_CraneModel, m_checkDrawModel);
	DDX_Control(pDX, IDC_CHECK_DrawDistance, m_checkDrawDistance);
	DDX_Text(pDX, IDC_EDIT_MinDistance, m_minDistance);
	DDX_Text(pDX, IDC_EDIT_maxDistance, m_maxDistance);
	DDX_Text(pDX, IDC_EDIT_numDistance, m_numDistance);
	DDX_Control(pDX, IDC_CHECK_PartExceptionRoi1, m_checkPartException1);
	DDX_Control(pDX, IDC_CHECK_PartExceptionRoi2, m_checkPartException2);
	DDX_Control(pDX, IDC_CHECK_PartExceptionRoi3, m_checkPartException3);
	DDX_Control(pDX, IDC_CHECK_PartExceptionRoi4, m_checkPartException4);
	DDX_Control(pDX, IDC_CHECK_PartExceptionRoi5, m_checkPartException5);
}

void CDistanceAnalyserDlg::OnDistance(SHI::Data::StDistance* pData)
{
	if (m_bWaiting)
	{
		UpdateDistance(pData);
	}
}

void CDistanceAnalyserDlg::UpdateDistance(SHI::Data::StDistance* pData)
{
	// update distance
	std::shared_ptr<SHI::Data::StDistance> data(new SHI::Data::StDistance);
	memcpy(data.get(), pData, sizeof(SHI::Data::StDistance));
	m_distance = data;	

	// update mfc gui
	PostMessage(WM_DATA_ANALYZER_UPDATE_GUI, 0, 0);

	// update pcl viewer
	RedrawViewer();
}

void OnKeyboardInput(const pcl::visualization::KeyboardEvent& event, void* param)
{
	pcl::visualization::PCLVisualizer* pViewer = (pcl::visualization::PCLVisualizer*)param;

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
	}
}

void CDistanceAnalyserDlg::RedrawViewer()
{
	m_eUpdateViewer.SetEvent();
}

void CDistanceAnalyserDlg::ThreadDraw()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("distance viewer"));
	std::shared_ptr<SHI::Data::StDistance> data;
	SHI::Utility::InitializeViewer(viewer);

	viewer->registerKeyboardCallback(OnKeyboardInput, viewer.get());
	
	// load crane model
	int32_t curPier = m_pierId;
	int32_t curCraneId = m_craneId;
	SHI::DistanceParamPtr distanceParam(new SHI::DistanceParam);
	SHI::CraneAttitude initialAttitude;
	SHI::PolygonModelVector vModels;
	SHI::GetDistanceParam(curPier, curCraneId, *distanceParam);
	SHI::GetInitialAttitude(curPier, curCraneId, initialAttitude);
	SHI::LoadPolygonModel(vModels, distanceParam->fnameModel);

	while (m_threadDraw.IsRunThread() && viewer->wasStopped() == false)
	{
		if (m_eUpdateViewer.WaitForEvent(10))
		{
			// get cluster data
			data = m_distance;

			// process draw
			if (data)
			{
				// extract data
				SHI::PointCloudPtr cloud = std::make_shared<SHI::PointCloud>();
				SHI::PCLIndicesVectorPtr indices(new SHI::PCLIndicesVector);
				std::vector<unsigned char> labels;
				SHI::DistanceInfoVectorPtr distance(new SHI::DistanceInfoVector);
				std::vector<unsigned char> distanceLabels;
				SHI::CraneAttitudePtr attitude(new SHI::CraneAttitude);
				SHI::Convert::DataDistance2Distance(data.get(), *cloud, *indices, labels, *distance, distanceLabels, *attitude);
				

				SHI::Utility::UpdateHookRoi(viewer, attitude);

				// update gui data
				if (m_checkDrawPoints.GetCheck())
				{
					SHI::Utility::UpdatePointCloud(viewer, cloud);
				}
				else
				{
					SHI::Utility::UpdatePointCloud(viewer);
				}

				if (m_checkDrawClusters.GetCheck())
				{
					SHI::Utility::UpdateCluster(viewer, cloud, indices, &labels);
					SHI::Utility::UpdateHookPoints(viewer, cloud, indices, &labels);
					SHI::Utility::UpdateGroundPoints(viewer, cloud, indices, &labels);
				}
				else
				{
					SHI::Utility::UpdateCluster(viewer);
					SHI::Utility::UpdateHookPoints(viewer);
					SHI::Utility::UpdateGroundPoints(viewer);
				}

				if (m_checkDrawDistance.GetCheck())
				{
					SHI::Utility::UpdateDistance(viewer, distance, &distanceLabels, m_maxDistance, m_numDistance);
				}
				else
				{
					SHI::Utility::UpdateDistance(viewer);
				}
				
				if (m_checkDrawModel.GetCheck())
				{
					// reload model
					if (attitude->pierId != curPier || attitude->craneId != curCraneId)
					{
						SHI::GetDistanceParam(attitude->pierId, attitude->craneId, *distanceParam);
						SHI::LoadPolygonModel(vModels, distanceParam->fnameModel);
					}

					// update model
					SHI::Utility::UpdatePolygonModel(viewer, attitude, &vModels);
				}
				else
				{
					SHI::Utility::UpdatePolygonModel(viewer, 0, 0);
				}
				
				if (m_checkDrawExceptionRoi.GetCheck())
				{
					SHI::Utility::UpdateRemoveDistanceRoi(viewer, distanceParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveDistanceRoi(viewer);
				}

				if (m_checkPartException1.GetCheck())
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 0, distanceParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 0);
				}

				if (m_checkPartException2.GetCheck())
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 1, distanceParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 1);
				}

				if (m_checkPartException3.GetCheck())
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 2, distanceParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 2);
				}

				if (m_checkPartException4.GetCheck())
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 3, distanceParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 3);
				}

				if (m_checkPartException5.GetCheck())
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 4, distanceParam, attitude);
				}
				else
				{
					SHI::Utility::UpdateRemoveDistancePartRoi(viewer, 4);
				}

				// bold point
				SHI::PointCloudPtr pointsBold = std::make_shared<SHI::PointCloud>();
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
				SHI::Utility::UpdatePointCloudBold(viewer, pointsBold);

				// bold distance
				std::vector<uint32_t> indicesDistanceBold;
				for (uint32_t i = 0; i < distance->size(); i++)
				{
					if (m_listDistance.GetCheck(i) == TRUE)
					{
						indicesDistanceBold.push_back(i);
					}
				}
				SHI::Utility::UpdateDistanceBold(viewer, distance, &indicesDistanceBold);
			}
		}
		viewer->spinOnce();
	}

	m_threadDraw.StopThread();
}

BEGIN_MESSAGE_MAP(CDistanceAnalyserDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_DATA_ANALYZER_UPDATE_GUI(OnUpdateGUI)
	ON_BN_CLICKED(IDC_BTN_ExportPcd, &CDistanceAnalyserDlg::OnBnClickedBtnExportpcd)
	ON_BN_CLICKED(IDC_BTN_SaveFile, &CDistanceAnalyserDlg::OnBnClickedBtnSavefile)
	ON_BN_CLICKED(IDC_BTN_LoadFile, &CDistanceAnalyserDlg::OnBnClickedBtnLoadfile)
	ON_BN_CLICKED(IDC_BTN_PrevFile, &CDistanceAnalyserDlg::OnBnClickedBtnPrevfile)
	ON_BN_CLICKED(IDC_BTN_NextFile, &CDistanceAnalyserDlg::OnBnClickedBtnNextfile)
	ON_BN_CLICKED(IDC_BTN_WriteMemory, &CDistanceAnalyserDlg::OnBnClickedBtnWritememory)
	ON_BN_CLICKED(IDC_BTN_ReadMemory, &CDistanceAnalyserDlg::OnBnClickedBtnReadmemory)
	ON_BN_CLICKED(IDC_BTN_WaitSync, &CDistanceAnalyserDlg::OnBnClickedBtnWaitsync)
	ON_BN_CLICKED(IDC_CHECK_Show3dWindow, &CDistanceAnalyserDlg::OnBnClickedCheckShow3dwindow)
	ON_BN_CLICKED(IDC_CHECK_DrawPoints, &CDistanceAnalyserDlg::OnBnClickedCheckDrawpoints)
	ON_BN_CLICKED(IDC_CHECK_DrawClusters, &CDistanceAnalyserDlg::OnBnClickedCheckDrawclusters)
	ON_BN_CLICKED(IDC_CHECK_ExceptionRoi, &CDistanceAnalyserDlg::OnBnClickedCheckExceptionroi)
	ON_BN_CLICKED(IDC_CHECK_CraneModel, &CDistanceAnalyserDlg::OnBnClickedCheckCranemodel)
	ON_BN_CLICKED(IDC_CHECK_DrawDistance, &CDistanceAnalyserDlg::OnBnClickedCheckDrawdistance)
	ON_NOTIFY(NM_CLICK, IDC_LIST_Clusters, &CDistanceAnalyserDlg::OnClickListClusters)
	ON_NOTIFY(NM_CLICK, IDC_LIST_Distance, &CDistanceAnalyserDlg::OnClickListDistance)
	ON_EN_CHANGE(IDC_EDIT_maxDistance, &CDistanceAnalyserDlg::OnEnChangeEditmaxdistance)
	ON_EN_CHANGE(IDC_EDIT_numDistance, &CDistanceAnalyserDlg::OnEnChangeEditnumdistance)
	ON_BN_CLICKED(IDC_CHECK_PartExceptionRoi1, &CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi1)
	ON_BN_CLICKED(IDC_CHECK_PartExceptionRoi2, &CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi2)
	ON_BN_CLICKED(IDC_CHECK_PartExceptionRoi3, &CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi3)
	ON_BN_CLICKED(IDC_CHECK_PartExceptionRoi4, &CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi4)
	ON_BN_CLICKED(IDC_CHECK_PartExceptionRoi5, &CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi5)
	ON_WM_DESTROY()//pjh
END_MESSAGE_MAP()


// CDistanceAnalyserDlg 메시지 처리기

BOOL CDistanceAnalyserDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	m_checkDrawPoints.SetCheck(TRUE);
	m_checkDrawClusters.SetCheck(TRUE);
	m_checkDrawExceptionRoi.SetCheck(TRUE);
	m_checkDrawModel.SetCheck(TRUE);
	m_checkDrawDistance.SetCheck(TRUE);
	m_checkPartException1.SetCheck(FALSE);
	m_checkPartException2.SetCheck(FALSE);
	m_checkPartException3.SetCheck(FALSE);
	m_checkPartException4.SetCheck(FALSE);
	m_checkPartException5.SetCheck(FALSE);
	m_eUpdateViewer.Create();
	
	m_listCluster.SetExtendedStyle(LVS_EX_CHECKBOXES);
	m_listCluster.InsertColumn(0, "index", LVCFMT_CENTER, 50);
	m_listCluster.InsertColumn(1, "nPoints", LVCFMT_CENTER, 70);
	m_listCluster.InsertColumn(2, "label", LVCFMT_CENTER, 50);
	
	m_listDistance.SetExtendedStyle(LVS_EX_CHECKBOXES);
	m_listDistance.InsertColumn(0, "index", LVCFMT_CENTER, 50);
	m_listDistance.InsertColumn(1, "distance", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(2, "label", LVCFMT_CENTER, 50);
	m_listDistance.InsertColumn(3, "part", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(4, "x", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(5, "y", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(6, "z", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(7, "cluster", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(8, "x", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(9, "y", LVCFMT_CENTER, 70);
	m_listDistance.InsertColumn(10, "z", LVCFMT_CENTER, 70);
	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CDistanceAnalyserDlg::OnPaint()
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
HCURSOR CDistanceAnalyserDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CDistanceAnalyserDlg::OnUpdateGUI(void)
{
	std::shared_ptr<SHI::Data::StDistance> distance = m_distance;
	if (distance)
	{
		UpdateData(TRUE);

		// update ctrls
		m_pierId = distance->attitude.pierId;
		m_craneId = distance->attitude.craneId;
		m_nPoints = distance->GetXYZSize();
		m_nClusters = distance->GetClusterInfoSize();
		m_nLabels = distance->GetLabelSize();

		// min distance
		float_t _minDistance = FLT_MAX;
		for (uint32_t i=0; i<distance->GetDistanceInfoSize(); i++)
		{
			// use only normal distance
			if (distance->GetDistanceLabel()[i] == SHI::DISTANCE_NORMAL)
			{
				_minDistance = (std::min)(_minDistance, distance->GetDistanceInfo()[i].Distance);
			}
		}
		m_minDistance = _minDistance;

		//update list cluster
		m_listCluster.DeleteAllItems();
		if (distance->GetClusterInfoSize() == distance->GetLabelSize())
		{
			for (uint32_t i = 0; i < distance->GetClusterInfoSize(); i++)
			{
				CString itemNameCluster, fieldCluster1, fieldCluster2, fieldCluster3;
				itemNameCluster.Format("item %d", i);
				fieldCluster1.Format("%d", i);
				fieldCluster2.Format("%d", distance->GetClusterInfo()[i].Size);
				fieldCluster3.Format("%d", distance->GetLabel()[i]);

				m_listCluster.InsertItem(i, itemNameCluster);
				m_listCluster.SetItemText(i, 0, fieldCluster1);
				m_listCluster.SetItemText(i, 1, fieldCluster2);
				m_listCluster.SetItemText(i, 2, fieldCluster3);
			}
		}

		//update list distance
		m_listDistance.DeleteAllItems();
		if (distance->GetDistanceInfoSize() == distance->GetDistanceLabelSize())
		{
			for (uint32_t i = 0; i < distance->GetDistanceInfoSize(); i++)
			{
				SHI::Data::StDistanceInfo info =  distance->GetDistanceInfo()[i];
				CString itemNameDistance, fieldIndex, fieldDistance[11];
				itemNameDistance.Format("item %d", i);
				fieldDistance[0].Format("%d", i);
				fieldDistance[1].Format("%.2f", info.Distance);
				fieldDistance[2].Format("%d", distance->GetDistanceLabel()[i]);
				fieldDistance[3].Format("%d", info.CraneIndex);
				fieldDistance[4].Format("%.2f", info.PosCrane.X);
				fieldDistance[5].Format("%.2f", info.PosCrane.Y);
				fieldDistance[6].Format("%.2f", info.PosCrane.Z);
				fieldDistance[7].Format("%d", info.ClusterIndex);
				fieldDistance[8].Format("%.2f", info.PosCluster.X);
				fieldDistance[9].Format("%.2f", info.PosCluster.Y);
				fieldDistance[10].Format("%.2f", info.PosCluster.Z);
		
				m_listDistance.InsertItem(i, itemNameDistance);
				for (uint32_t idx=0; idx<11; idx++)
				{
					m_listDistance.SetItemText(i, idx, fieldDistance[idx]);
				}
			}
		}

		UpdateData(FALSE);
	}
}

void CDistanceAnalyserDlg::OnBnClickedBtnExportpcd()
{
	std::shared_ptr<SHI::Data::StDistance> distance = m_distance;
	if (distance)
	{
		char filter[] = "pcd Files (*.pcd)|*.pcd|";
		CFileDialog fileDlg(FALSE, "pcd file", "*.pcd", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, filter, NULL);
		if (fileDlg.DoModal() == IDOK)
		{
			CString fname = fileDlg.GetPathName();

			// cluster to pcd
			SHI::PointCloud cloud;
			SHI::PCLIndicesVector indices;
			std::vector<unsigned char> labels;
			SHI::DistanceInfoVector dist;
			std::vector<unsigned char> distanceLabels;
			SHI::CraneAttitudePtr attitude(new SHI::CraneAttitude);
			SHI::Convert::DataDistance2Distance(distance.get(), cloud, indices, labels, dist, distanceLabels, *attitude);

			// save pcd
			//pcl::io::savePCDFileBinary(std::string((LPCTSTR)fname), cloud);
		}
	}
}


void CDistanceAnalyserDlg::OnBnClickedBtnSavefile()
{
	std::shared_ptr<SHI::Data::StDistance> distance = m_distance;
	if (distance)
	{
		char filter[] = "StDistance Files (*.StDistance)|*.StDistance|";
		CFileDialog fileDlg(FALSE, "StDistance", "*.StDistance", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, filter, NULL);
		if (fileDlg.DoModal() == IDOK)
		{
			// save cluster
			CString fname = fileDlg.GetPathName();
			SHI::FileUtility::SaveDistanceFile(*distance, std::string((LPCTSTR)fname));
		}
	}
}


void CDistanceAnalyserDlg::OnBnClickedBtnLoadfile()
{
	std::shared_ptr<SHI::Data::StDistance> distance = std::shared_ptr<SHI::Data::StDistance>(new SHI::Data::StDistance);

	CFileDialog fileDlg(TRUE, NULL, "distance.StDistance", OFN_HIDEREADONLY | OFN_NOCHANGEDIR, "Distance Files (*.StDistance)|*.StDistance||");
	if (fileDlg.DoModal() == IDOK)
	{
		CString fname = fileDlg.GetPathName();
		if (SHI::FileUtility::LoadDistanceFile(*distance, std::string((LPCTSTR)fname)))
		{
			// load cluster
			UpdateDistance(distance.get());

			// update directory info
			m_txtCurrentData = fileDlg.GetFileName();
			m_selectedFile = fileDlg.GetPathName();
			m_selectedFolder = fileDlg.GetFolderPath();
		}
	}
	UpdateData(FALSE);
}


void CDistanceAnalyserDlg::OnBnClickedBtnPrevfile()
{
	if (!m_selectedFolder.empty())
	{
		std::vector<std::string> vDistanceFiles;
		FindDistanceFiles(vDistanceFiles, m_selectedFolder.c_str(), "StDistance");

		if (vDistanceFiles.size())
		{
			std::string newFile;

			// 현재 선택된 폴더 탐색하여 다음 파일을 찾음
			for (int32_t i = 0; i < vDistanceFiles.size(); i++)
			{
				if (vDistanceFiles[i].compare(m_selectedFile) == 0 && (i - 1) >= 0)
				{
					newFile = vDistanceFiles[i - 1];
					break;
				}
			}

			// 다음 파일이 있으면 해당 데이터 불러와서 적용
			if (!newFile.empty())
			{
				SHI::Data::StDistance *pData = new SHI::Data::StDistance;
				if (SHI::FileUtility::LoadDistanceFile(*pData, newFile.c_str()))
				{
					m_selectedFile = newFile;
					m_txtCurrentData = newFile.c_str();

					int32_t s = newFile.find_last_of('\\');
					if (s != -1)
					{
						m_txtCurrentData = newFile.substr(s + 1, newFile.size()).c_str();
						UpdateDistance(pData);
					}
					UpdateData(FALSE);
				}
			}
		}
	}
}


void CDistanceAnalyserDlg::OnBnClickedBtnNextfile()
{
	if (!m_selectedFolder.empty())
	{
		std::vector<std::string> vDistanceFiles;
		FindDistanceFiles(vDistanceFiles, m_selectedFolder.c_str(), "StDistance");

		if (vDistanceFiles.size())
		{
			std::string newFile;

			// 현재 선택된 폴더 탐색하여 다음 파일을 찾음
			for (int32_t i = 0; i < vDistanceFiles.size(); i++)
			{
				if (vDistanceFiles[i].compare(m_selectedFile) == 0 && (i + 1) < vDistanceFiles.size())
				{
					newFile = vDistanceFiles[i + 1];
					break;
				}
			}

			// 다음 파일이 있으면 해당 데이터 불러와서 적용
			if (!newFile.empty())
			{
				SHI::Data::StDistance *pData = new SHI::Data::StDistance;
				if (SHI::FileUtility::LoadDistanceFile(*pData, newFile.c_str()))
				{
					m_selectedFile = newFile;
					m_txtCurrentData = newFile.c_str();

					int32_t s = newFile.find_last_of('\\');
					if (s != -1)
					{
						m_txtCurrentData = newFile.substr(s + 1, newFile.size()).c_str();
						UpdateDistance(pData);
					}
					UpdateData(FALSE);
				}
			}
		}
	}
}


void CDistanceAnalyserDlg::OnBnClickedBtnWritememory()
{
	if (m_distance)
	{
		UpdateData(TRUE);
		SHI::Interface::Distance::Stub::CStubDistance stub;
		stub.GetStubDistance()->Create(m_id);
		stub.GetStubDistance()->WriteData(m_distance.get());
	}
}


void CDistanceAnalyserDlg::OnBnClickedBtnReadmemory()
{
	UpdateData(TRUE);
	UpdateData(FALSE);
	SHI::Data::StDistance* pData = new SHI::Data::StDistance;
	static CProxyDistanceObject obj;
	obj.GetProxyDistance()->Create(m_id);
	if (obj.GetProxyDistance()->ReadBuffer(pData))
	{
		UpdateDistance(pData);
	}
	delete pData;
}


void CDistanceAnalyserDlg::OnBnClickedBtnWaitsync()
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
		GetProxyDistance()->Create(m_id);
		SetDlgItemText(IDC_BTN_WaitSync, "Stop Sync");
	}
}


void CDistanceAnalyserDlg::OnBnClickedCheckShow3dwindow()
{
	if (m_checkShow3DWindow.GetCheck() == TRUE)
	{
		m_threadDraw.StartThread(&CDistanceAnalyserDlg::ThreadDraw, this);
		RedrawViewer();
	}
	else
	{
		m_threadDraw.StopThread();
	}
}


void CDistanceAnalyserDlg::OnBnClickedCheckDrawpoints()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckDrawclusters()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckExceptionroi()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckCranemodel()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckDrawdistance()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnClickListClusters(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	if (pNMItemActivate->iItem >= 0)
	{
		RedrawViewer();
	}
	*pResult = 0;
}


void CDistanceAnalyserDlg::OnClickListDistance(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	if (pNMItemActivate->iItem >= 0)
	{
		RedrawViewer();
	}
	*pResult = 0;
}


void CDistanceAnalyserDlg::OnEnChangeEditmaxdistance()
{
	UpdateData(TRUE);
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnEnChangeEditnumdistance()
{
	UpdateData(TRUE);
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi1()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi2()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi3()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi4()
{
	RedrawViewer();
}


void CDistanceAnalyserDlg::OnBnClickedCheckPartexceptionroi5()
{
	RedrawViewer();
}
