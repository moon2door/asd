
// IntrinsicCalibrationDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "IntrinsicCalibration.h"
#include "IntrinsicCalibrationDlg.h"
#include "afxdialogex.h"

#include <Utility/mathematics.h>
#include <Utility/Config.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CIntrinsicCalibrationDlg 대화 상자

/// @brief IntrinsicCalibarationDialog 생성자
// 각 인자들 초기화
/// @param pParent
CIntrinsicCalibrationDlg::CIntrinsicCalibrationDlg(CWnd* pParent /*=NULL*/)
    : CDialogEx(IDD_INTRINSICCALIBRATION_DIALOG, pParent), m_viewer(nullptr), m_radioLidar(0), m_rx(0), m_ry(0), m_rz(0), m_x(0), m_y(0), m_z(0), m_drx(0.001f), m_dry(0.001f), m_drz(0.001f), m_dx(0.001f), m_dy(0.001f), m_dz(0.001f)
{
    m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

    m_minParams = {-4.0f, -4.0f, -4.0f, -0.05f, -0.05f, -0.05f};
    m_maxParams = {4.0f, 4.0f, 4.0f, 0.05f, 0.05f, 0.05f};
    m_sliderMin = -1000;
    m_sliderMax = 1000;
}

void CIntrinsicCalibrationDlg::DoDataExchange(CDataExchange *pDX)
{
    CDialogEx::DoDataExchange(pDX);
    DDX_Radio(pDX, IDC_RADIO1, m_radioLidar);
    DDX_Text(pDX, IDC_EDIT_RX, m_rx); // 뷰 개체의 데이터 멤버간의 데이터 전송 관리
    DDX_Text(pDX, IDC_EDIT_RY, m_ry);
    DDX_Text(pDX, IDC_EDIT_RZ, m_rz);
    DDX_Text(pDX, IDC_EDIT_X, m_x);
    DDX_Text(pDX, IDC_EDIT_Y, m_y);
    DDX_Text(pDX, IDC_EDIT_Z, m_z);
    DDX_Text(pDX, IDC_EDIT_DRX, m_drx);
    DDX_Text(pDX, IDC_EDIT_DRY, m_dry);
    DDX_Text(pDX, IDC_EDIT_DRZ, m_drz);
    DDX_Text(pDX, IDC_EDIT_DX, m_dx);
    DDX_Text(pDX, IDC_EDIT_DY, m_dy);
    DDX_Text(pDX, IDC_EDIT_DZ, m_dz);
    DDX_Control(pDX, IDC_SLIDER_RX, m_sliderRx); // DDX_Control(CDataExchange* , 자원의 ID, 자원과 연결시킬 클래스 변수) // m_sliderRx를 IDC_SLIDER_RX와 연결
    DDX_Control(pDX, IDC_SLIDER_RY, m_sliderRy);
    DDX_Control(pDX, IDC_SLIDER_RZ, m_sliderRz);
    DDX_Control(pDX, IDC_SLIDER_X, m_sliderX);
    DDX_Control(pDX, IDC_SLIDER_Y, m_sliderY);
    DDX_Control(pDX, IDC_SLIDER_Z, m_sliderZ);
}

BEGIN_MESSAGE_MAP(CIntrinsicCalibrationDlg, CDialogEx)                                  // 메세지 맵의 정의를 시작
ON_WM_PAINT()                                                                           // 윈도우에 그림 그리는 메세지를 보낼 때 호출
ON_WM_QUERYDRAGICON()                                                                   // 사용자가 최소화된 창을 드래그할 때 보여줄 아이콘의 핸들을 얻기 위해 호출
ON_BN_CLICKED(IDC_BUTTON_LOAD_LOG, &CIntrinsicCalibrationDlg::OnBnClickedButtonLoadLog) // 버튼이 클릭되면 특정 함수가 호출
ON_BN_CLICKED(IDC_BUTTON_LOAD_PARAM, &CIntrinsicCalibrationDlg::OnBnClickedButtonLoadParam)
ON_BN_CLICKED(IDC_BUTTON_SAVE_PARAM, &CIntrinsicCalibrationDlg::OnBnClickedButtonSaveParam)
ON_WM_HSCROLL()                                                        // 이 창에서 사용자가 가로로 스코랄을 할 때 호출
ON_EN_CHANGE(IDC_EDIT_RX, &CIntrinsicCalibrationDlg::OnEnChangeEditRx) // 텍스트 박스의 내용이 변경되면 특정 함수를 호출
ON_EN_CHANGE(IDC_EDIT_RY, &CIntrinsicCalibrationDlg::OnEnChangeEditRy)
ON_EN_CHANGE(IDC_EDIT_RZ, &CIntrinsicCalibrationDlg::OnEnChangeEditRz)
ON_EN_CHANGE(IDC_EDIT_X, &CIntrinsicCalibrationDlg::OnEnChangeEditX)
ON_EN_CHANGE(IDC_EDIT_Y, &CIntrinsicCalibrationDlg::OnEnChangeEditY)
ON_EN_CHANGE(IDC_EDIT_Z, &CIntrinsicCalibrationDlg::OnEnChangeEditZ)
ON_BN_CLICKED(IDC_BUTTON_RX_MINUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonRxMinus)
ON_BN_CLICKED(IDC_BUTTON_RX_PLUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonRxPlus)
ON_BN_CLICKED(IDC_BUTTON_RY_MINUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonRyMinus)
ON_BN_CLICKED(IDC_BUTTON_RY_PLUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonRyPlus)
ON_BN_CLICKED(IDC_BUTTON_RZ_MINUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonRzMinus)
ON_BN_CLICKED(IDC_BUTTON_RZ_PLUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonRzPlus)
ON_BN_CLICKED(IDC_BUTTON_X_MINUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonXMinus)
ON_BN_CLICKED(IDC_BUTTON_X_PLUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonXPlus)
ON_BN_CLICKED(IDC_BUTTON_Y_MINUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonYMinus)
ON_BN_CLICKED(IDC_BUTTON_Y_PLUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonYPlus)
ON_BN_CLICKED(IDC_BUTTON_Z_MINUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonZMinus)
ON_BN_CLICKED(IDC_BUTTON_Z_PLUS, &CIntrinsicCalibrationDlg::OnBnClickedButtonZPlus)
ON_BN_CLICKED(IDC_BUTTON_SAVEPOINT, &CIntrinsicCalibrationDlg::OnBnClickedButtonSavepoint)
ON_BN_CLICKED(IDC_BUTTON_SELECTLIDAR, &CIntrinsicCalibrationDlg::OnBnClickedButtonSelectLidar)
END_MESSAGE_MAP() // 메세지 맵의 정의 종료

// CIntrinsicCalibrationDlg 메시지 처리기

BOOL CIntrinsicCalibrationDlg::OnInitDialog()
{
    CDialogEx::OnInitDialog();

    // 이 대화 상자의 아이콘을 설정합니다.  응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
    //  프레임워크가 이 작업을 자동으로 수행합니다.
    SetIcon(m_hIcon, TRUE);  // 큰 아이콘을 설정합니다.
    SetIcon(m_hIcon, FALSE); // 작은 아이콘을 설정합니다.

    // TODO: 여기에 추가 초기화 작업을 추가합니다.
    m_sliderRx.SetRangeMin(m_sliderMin);
    m_sliderRy.SetRangeMin(m_sliderMin);
    m_sliderRz.SetRangeMin(m_sliderMin);
    m_sliderX.SetRangeMin(m_sliderMin);
    m_sliderY.SetRangeMin(m_sliderMin);
    m_sliderZ.SetRangeMin(m_sliderMin);

    m_sliderRx.SetRangeMax(m_sliderMax);
    m_sliderRy.SetRangeMax(m_sliderMax);
    m_sliderRz.SetRangeMax(m_sliderMax);
    m_sliderX.SetRangeMax(m_sliderMax);
    m_sliderY.SetRangeMax(m_sliderMax);
    m_sliderZ.SetRangeMax(m_sliderMax);

    m_sliderRx.SetPos(50);
    m_sliderRy.SetPos(50);
    m_sliderRz.SetPos(50);
    m_sliderX.SetPos(50);
    m_sliderY.SetPos(50);
    m_sliderZ.SetPos(50);

    m_glAxisObject.SetType(Routine::GL::ObjectType::GlLines);
    m_glAxisObject.SetDrawPointSize(2);
    m_glAxisObject.AddPoint(10, 0, 0);
    m_glAxisObject.AddPoint(0, 0, 0);
    m_glAxisObject.AddPoint(0, 10, 0);
    m_glAxisObject.AddPoint(0, 0, 0);
    m_glAxisObject.AddPoint(0, 0, 10);
    m_glAxisObject.AddPoint(0, 0, 0);
    m_glAxisObject.AddColor(1, 0, 0);
    m_glAxisObject.AddColor(1, 0, 0);
    m_glAxisObject.AddColor(0, 1, 0);
    m_glAxisObject.AddColor(0, 1, 0);
    m_glAxisObject.AddColor(0, 0, 1);
    m_glAxisObject.AddColor(0, 0, 1);

    
    //pjh
    //m_viewer.SetZoom(50);
    //

    return TRUE; // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다.  문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CIntrinsicCalibrationDlg::OnPaint()
{
    if (IsIconic())
    {
        CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

        // 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
        const int cxIcon = GetSystemMetrics(SM_CXICON);
        const int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        const int x = (rect.Width() - cxIcon + 1) / 2;
        const int y = (rect.Height() - cyIcon + 1) / 2;

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
HCURSOR CIntrinsicCalibrationDlg::OnQueryDragIcon()
{
    return static_cast<HCURSOR>(m_hIcon);
}

void CIntrinsicCalibrationDlg::UpdateSliders()
{
    UpdateData(TRUE);
    int pos = m_sliderMin + (m_rx - m_minParams[0]) * (m_sliderMax - m_sliderMin) / (m_maxParams[0] - m_minParams[0]);
    m_sliderRx.SetPos(pos);
    pos = m_sliderMin + (m_ry - m_minParams[1]) * (m_sliderMax - m_sliderMin) / (m_maxParams[1] - m_minParams[1]);
    m_sliderRy.SetPos(pos);
    pos = m_sliderMin + (m_rz - m_minParams[2]) * (m_sliderMax - m_sliderMin) / (m_maxParams[2] - m_minParams[2]);
    m_sliderRz.SetPos(pos);
    pos = m_sliderMin + (m_x - m_minParams[3]) * (m_sliderMax - m_sliderMin) / (m_maxParams[3] - m_minParams[3]);
    m_sliderX.SetPos(pos);
    pos = m_sliderMin + (m_y - m_minParams[4]) * (m_sliderMax - m_sliderMin) / (m_maxParams[4] - m_minParams[4]);
    m_sliderY.SetPos(pos);
    pos = m_sliderMin + (m_z - m_minParams[5]) * (m_sliderMax - m_sliderMin) / (m_maxParams[5] - m_minParams[5]);
    m_sliderZ.SetPos(pos);
}

Routine::CPointCloud<Routine::CPoint4Df>::SharedPtr ProcessConvertXYZAzimuth(const std::shared_ptr<std::deque<Pala720Packet>> &packetBuffer, const Routine::CMatrix44f &intrinsic)
{
    auto pointCloudResult = std::make_shared<Routine::CPointCloud<Routine::CPoint4Df>>();
    pointCloudResult->reserve(500000);
    for (const auto &packet : *packetBuffer)
    {
        const auto &lidar = packet.lidarPacket;
        const auto &rotor = packet.rotorPacket;

        for (const auto &laser : lidar.lidarSignals)
        {
            const float_t cosTheta = cos(laser.azimuth_deg * SHI::Math::D2Rf);
            const float_t sinTheta = sin(laser.azimuth_deg * SHI::Math::D2Rf);
            const float_t cosPhi = cos(laser.altitude_deg * SHI::Math::D2Rf);
            const float_t sinPhi = sin(laser.altitude_deg * SHI::Math::D2Rf);

            const float_t x = laser.range_m * sinTheta * cosPhi;
            const float_t y = laser.range_m * cosTheta * cosPhi;
            const float_t z = laser.range_m * sinPhi;

            const float_t xBase = intrinsic(0, 0) * x + intrinsic(0, 1) * (-z) + intrinsic(0, 2) * y + intrinsic(0, 3);
            const float_t yBase = intrinsic(1, 0) * x + intrinsic(1, 1) * (-z) + intrinsic(1, 2) * y + intrinsic(1, 3);
            const float_t zBase = intrinsic(2, 0) * x + intrinsic(2, 1) * (-z) + intrinsic(2, 2) * y + intrinsic(2, 3);

            const auto rotorAngle = static_cast<float_t>(rotor.angle);
            const float_t cosRotor = cos(rotorAngle * SHI::Math::D2Rf);
            const float_t sinRotor = sin(rotorAngle * SHI::Math::D2Rf);
            const float_t xRotor = cosRotor * xBase - sinRotor * yBase;
            const float_t yRotor = sinRotor * xBase + cosRotor * yBase;
            const float_t zRotor = zBase;
            pointCloudResult->push_back(Routine::CPoint4Df(-xRotor, yRotor, zRotor, laser.azimuth_deg));
        }
    }
    return pointCloudResult;
}

Routine::CPointCloud<Routine::CPoint4Df>::SharedPtr ProcessConvertXYZI(const std::shared_ptr<std::deque<Pala720Packet>> &packetBuffer, const Routine::CMatrix44f &intrinsic)
{
    auto pointCloudResult = std::make_shared<Routine::CPointCloud<Routine::CPoint4Df>>();
    pointCloudResult->reserve(500000);
    for (const auto &packet : *packetBuffer)
    {
        const auto &lidar = packet.lidarPacket;
        const auto &rotor = packet.rotorPacket;

        for (const auto &laser : lidar.lidarSignals)
        {
            const float_t cosTheta = cos(laser.azimuth_deg * SHI::Math::D2Rf);
            const float_t sinTheta = sin(laser.azimuth_deg * SHI::Math::D2Rf);
            const float_t cosPhi = cos(laser.altitude_deg * SHI::Math::D2Rf);
            const float_t sinPhi = sin(laser.altitude_deg * SHI::Math::D2Rf);

            const float_t x = laser.range_m * sinTheta * cosPhi;
            const float_t y = laser.range_m * cosTheta * cosPhi;
            const float_t z = laser.range_m * sinPhi;

            const float_t xBase = intrinsic(0, 0) * x + intrinsic(0, 1) * (-z) + intrinsic(0, 2) * y + intrinsic(0, 3);
            const float_t yBase = intrinsic(1, 0) * x + intrinsic(1, 1) * (-z) + intrinsic(1, 2) * y + intrinsic(1, 3);
            const float_t zBase = intrinsic(2, 0) * x + intrinsic(2, 1) * (-z) + intrinsic(2, 2) * y + intrinsic(2, 3);

            const auto rotorAngle = static_cast<float_t>(rotor.angle);
            const float_t cosRotor = cos(rotorAngle * SHI::Math::D2Rf);
            const float_t sinRotor = sin(rotorAngle * SHI::Math::D2Rf);
            const float_t xRotor = cosRotor * xBase - sinRotor * yBase;
            const float_t yRotor = sinRotor * xBase + cosRotor * yBase;
            const float_t zRotor = zBase;
            pointCloudResult->push_back(Routine::CPoint4Df(-xRotor, yRotor, zRotor, laser.intensity));
        }
    }
    return pointCloudResult;
}

void CIntrinsicCalibrationDlg::UpdateViewer()
{
    UpdateData(TRUE);

    if (m_packetBuffer)
    {
        Routine::CMatrix33f rotation;
        rotation =
            Eigen::AngleAxisf(m_rx * SHI::Math::D2Rf, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(m_ry * SHI::Math::D2Rf, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(m_rz * SHI::Math::D2Rf, Eigen::Vector3f::UnitZ());
        const Routine::CPoint3Df translation(m_x, m_y, m_z);
        Routine::CMatrix44f intrinsicMatrix = Routine::CMatrix44f::Identity();
        intrinsicMatrix.topLeftCorner(3, 3) = rotation;
        intrinsicMatrix.topRightCorner(3, 1) = translation;

        const auto pointCloud = ProcessConvertXYZAzimuth(m_packetBuffer, intrinsicMatrix);

        Routine::GL::CGlObject glPointObject;
        glPointObject.SetType(Routine::GL::ObjectType::GlPoints);
        glPointObject.GetPoints().reserve(pointCloud->size());
        glPointObject.GetColors().reserve(pointCloud->size());
        for (const auto &p : pointCloud->GetPoints())
        {
            auto color = Routine::Graphics::CImageColor::GetLUTColor(static_cast<uint32_t>(p.w() + 5), 370);
            glPointObject.GetColors().emplace_back(color.r() / 255.f, color.g() / 255.f, color.b() / 255.f, 1.0);
            glPointObject.GetPoints().emplace_back(p.x(), p.y(), p.z());
        }

        Routine::GL::CGlModel glModel;
        glModel.AddObject(m_glAxisObject);
        glModel.AddObject(glPointObject);
        
        //m_viewer.ShowModel(glModel);
        m_viewer->ShowModel(glModel);
    }
}

std::shared_ptr<std::deque<Pala720Packet>> ParseLog(const std::string &fileName, int32_t sensorId, size_t maxSeconds)
{
    auto outPacket = std::make_shared<std::deque<Pala720Packet>>();
    FILE *fp = nullptr;
    if (fopen_s(&fp, fileName.c_str(), "rb") == 0)
    {
        printf("Read log file: %s\n", fileName.c_str());

        int64_t size = 0;
        size_t count = 0;
        while (sizeof(size) == fread(&size, 1, sizeof(size), fp))
        {
            if (size > 0)
            {
                std::vector<uint8_t> buffer;
                buffer.resize(size);
                int64_t remained = size;
                while (remained > 0)
                {
                    const auto nRead = fread(buffer.data(), 1, remained, fp);
                    remained -= static_cast<int64_t>(nRead);
                }

                Routine::CDeSerializer deSerializer(buffer.data());

                Pala720Packet packet;
                deSerializer >> packet;
                if (packet.id == sensorId)
                {
                    outPacket->push_back(packet);
                }
            }
            else
            {
                count++;
            }

            if (count > maxSeconds)
                break;
        }

        fclose(fp);
    }
    return outPacket;
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonLoadLog()
{
    UpdateData(TRUE);
    const CString str = "로그 파일 (*.log) |*.log|"; // 모든 파일 표시
    CFileDialog dlg(TRUE, "*.log", nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, str, this);
    
    //pjh
    m_viewer = std::make_shared<Routine::Visualization::CPointCloudViewer>();
    m_viewer->SetZoom(50);
    //
    
    if (dlg.DoModal() == IDOK)
    {
        const CString strPathName = dlg.GetPathName();
        SetDlgItemText(IDC_EDIT1, strPathName);
        folderPath = dlg.GetFolderPath().GetString();
        fileName = strPathName.GetString();
        configFileName = folderPath + "\\interfaceRotor.json";

        const int32_t id = m_radioLidar;
        char szFieldName[256] = "";
        sprintf_s(szFieldName, "Rotor%d", (id + 1));
        Routine::CMatrix44f intrinsicMatrix = Utility::GetConfigMatrix44f(szFieldName, "intrinsicParameters", configFileName);
        const Routine::CMatrix33f rotationMatrix = intrinsicMatrix.topLeftCorner(3, 3);
        const Routine::CPoint3Df rotation = rotationMatrix.eulerAngles(0, 1, 2);
        const Routine::CPoint3Df translation = intrinsicMatrix.topRightCorner(3, 1);

        m_x = translation(0);
        m_y = translation(1);
        m_z = translation(2);
        m_rx = rotation(0) * SHI::Math::R2Df;
        m_ry = rotation(1) * SHI::Math::R2Df;
        m_rz = rotation(2) * SHI::Math::R2Df;

        m_packetBuffer = ParseLog(fileName, id, 3);
        m_packetBufferAll = ParseLog(fileName, id, -1);
        UpdateData(FALSE);
        UpdateViewer();
    }
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonLoadParam()
{
    const CString str = "설정 파일 (*.json) |*.json|";
    CFileDialog dlg(TRUE, "*.json", nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, str, this);

    if (dlg.DoModal() == IDOK)
    {
        const std::string configFileName = dlg.GetPathName().GetString();

        const int32_t id = m_radioLidar;
        char szFieldName[256] = "";
        sprintf_s(szFieldName, "Rotor%d", (id + 1));
        Routine::CMatrix44f intrinsicMatrix = Utility::GetConfigMatrix44f(szFieldName, "intrinsicParameters", configFileName);
        const Routine::CMatrix33f rotationMatrix = intrinsicMatrix.topLeftCorner(3, 3);
        const Routine::CPoint3Df rotation = rotationMatrix.eulerAngles(0, 1, 2);
        const Routine::CPoint3Df translation = intrinsicMatrix.topRightCorner(3, 1);

        m_x = translation(0);
        m_y = translation(1);
        m_z = translation(2);
        m_rx = rotation(0) * SHI::Math::R2Df;
        m_ry = rotation(1) * SHI::Math::R2Df;
        m_rz = rotation(2) * SHI::Math::R2Df;

        UpdateData(FALSE);

        UpdateSliders();
        UpdateViewer();
    }
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonSaveParam()
{
    CString str = "설정 파일 (*.json) |*.json|";

    CFileDialog dlg(FALSE, "json", "*.json", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, str, nullptr);
    dlg.m_ofn.nFilterIndex = 1;

    // 다이얼로그를 띄운다.
    if (dlg.DoModal() == IDOK)
    {
        std::string strPathName = dlg.GetPathName().GetString();

        Routine::CMatrix33f rotation;
        rotation =
            Eigen::AngleAxisf(m_rx * SHI::Math::D2Rf, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(m_ry * SHI::Math::D2Rf, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(m_rz * SHI::Math::D2Rf, Eigen::Vector3f::UnitZ());
        Routine::CPoint3Df translation(m_x, m_y, m_z);
        Routine::CMatrix44f intrinsicMatrix = Routine::CMatrix44f::Identity();
        intrinsicMatrix.topLeftCorner(3, 3) = rotation;
        intrinsicMatrix.topRightCorner(3, 1) = translation;

        const int32_t id = m_radioLidar;
        char szFieldName[256] = "";
        sprintf_s(szFieldName, "Rotor%d", (id + 1));
        Utility::SetConfigMatrix44f(szFieldName, "intrinsicParameters", intrinsicMatrix, strPathName);
    }
}

void CIntrinsicCalibrationDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar *pScrollBar)
{
    // TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
    if (pScrollBar)
    {
        if (pScrollBar == (CScrollBar *)&m_sliderRx)
        {
            int32_t position = m_sliderRx.GetPos();
            m_rx = m_minParams[0] + (position - m_sliderMin) * (m_maxParams[0] - m_minParams[0]) / (m_sliderMax - m_sliderMin);
        }
        if (pScrollBar == (CScrollBar *)&m_sliderRy)
        {
            int32_t position = m_sliderRy.GetPos();
            m_ry = m_minParams[1] + (position - m_sliderMin) * (m_maxParams[1] - m_minParams[1]) / (m_sliderMax - m_sliderMin);
        }
        if (pScrollBar == (CScrollBar *)&m_sliderRz)
        {
            int32_t position = m_sliderRz.GetPos();
            m_rz = m_minParams[2] + (position - m_sliderMin) * (m_maxParams[2] - m_minParams[2]) / (m_sliderMax - m_sliderMin);
        }
        if (pScrollBar == (CScrollBar *)&m_sliderX)
        {
            int32_t position = m_sliderX.GetPos();
            m_x = m_minParams[3] + (position - m_sliderMin) * (m_maxParams[3] - m_minParams[3]) / (m_sliderMax - m_sliderMin);
        }
        if (pScrollBar == (CScrollBar *)&m_sliderY)
        {
            int32_t position = m_sliderY.GetPos();
            m_y = m_minParams[4] + (position - m_sliderMin) * (m_maxParams[4] - m_minParams[4]) / (m_sliderMax - m_sliderMin);
        }
        if (pScrollBar == (CScrollBar *)&m_sliderZ)
        {
            int32_t position = m_sliderZ.GetPos();
            m_z = m_minParams[5] + (position - m_sliderMin) * (m_maxParams[5] - m_minParams[5]) / (m_sliderMax - m_sliderMin);
        }
        else
        {
        }
        UpdateData(FALSE);
        UpdateViewer();
    }
    CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}

void CIntrinsicCalibrationDlg::OnEnChangeEditRx()
{
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnEnChangeEditRy()
{
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnEnChangeEditRz()
{
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnEnChangeEditX()
{
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnEnChangeEditY()
{
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnEnChangeEditZ()
{
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonRxMinus()
{
    UpdateData(TRUE);
    m_rx -= m_drx;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonRxPlus()
{
    UpdateData(TRUE);
    m_rx += m_drx;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonRyMinus()
{
    UpdateData(TRUE);
    m_ry -= m_dry;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonRyPlus()
{
    UpdateData(TRUE);
    m_ry += m_dry;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonRzMinus()
{
    UpdateData(TRUE);
    m_rz -= m_drz;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonRzPlus()
{
    UpdateData(TRUE);
    m_rz += m_drz;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonXMinus()
{
    UpdateData(TRUE);
    m_x -= m_dx;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonXPlus()
{
    UpdateData(TRUE);
    m_x += m_dx;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonYMinus()
{
    UpdateData(TRUE);
    m_y -= m_dy;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonYPlus()
{
    UpdateData(TRUE);
    m_y += m_dy;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonZMinus()
{
    UpdateData(TRUE);
    m_z -= m_dz;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonZPlus()
{
    UpdateData(TRUE);
    m_z += m_dz;
    UpdateData(FALSE);
    UpdateSliders();
    UpdateViewer();
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonSavepoint()
{
    UpdateData(TRUE);
    CString str = "포인트 클라우드 파일 (*.csv) |*.csv|";

    CFileDialog dlg(FALSE, "csv", "*.csv", OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR, str, nullptr);
    dlg.m_ofn.nFilterIndex = 1;

    // 다이얼로그를 띄운다.
    if (dlg.DoModal() == IDOK)
    {
        CString strPathName = dlg.GetPathName();

        if (m_packetBuffer)
        {
            Routine::CMatrix33f rotation;
            rotation =
                Eigen::AngleAxisf(m_rx * SHI::Math::D2Rf, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(m_ry * SHI::Math::D2Rf, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(m_rz * SHI::Math::D2Rf, Eigen::Vector3f::UnitZ());
            const Routine::CPoint3Df translation(m_x, m_y, m_z);
            Routine::CMatrix44f intrinsicMatrix = Routine::CMatrix44f::Identity();
            intrinsicMatrix.topLeftCorner(3, 3) = rotation;
            intrinsicMatrix.topRightCorner(3, 1) = translation;

            const auto pointCloud = ProcessConvertXYZAzimuth(m_packetBufferAll, intrinsicMatrix);

            FILE *fp = nullptr;
            if (fopen_s(&fp, strPathName.GetBuffer(), "wt") == 0)
            {
                for (const auto &point : pointCloud->GetPoints())
                {
                    fprintf_s(fp, "%f, %f, %f, %f\n", point.x(), point.y(), point.z(), point.w());
                }
                fclose(fp);
            }
        }
    }
}

void CIntrinsicCalibrationDlg::OnBnClickedButtonSelectLidar()
{
    UpdateData(TRUE);

    const int32_t id = m_radioLidar;
    char szFieldName[256] = "";
    sprintf_s(szFieldName, "Rotor%d", (id + 1));
    Routine::CMatrix44f intrinsicMatrix = Utility::GetConfigMatrix44f(szFieldName, "intrinsicParameters", configFileName);
    const Routine::CMatrix33f rotationMatrix = intrinsicMatrix.topLeftCorner(3, 3);
    const Routine::CPoint3Df rotation = rotationMatrix.eulerAngles(0, 1, 2);
    const Routine::CPoint3Df translation = intrinsicMatrix.topRightCorner(3, 1);

    m_x = translation(0);
    m_y = translation(1);
    m_z = translation(2);
    m_rx = rotation(0) * SHI::Math::R2Df;
    m_ry = rotation(1) * SHI::Math::R2Df;
    m_rz = rotation(2) * SHI::Math::R2Df;

    m_packetBuffer = ParseLog(fileName, id, 3);
    m_packetBufferAll = ParseLog(fileName, id, -1);
    UpdateData(FALSE);
    UpdateViewer();
}

BOOL CIntrinsicCalibrationDlg::PreTranslateMessage(MSG *pMsg)
{
    // TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

    if (WM_KEYDOWN == pMsg->message)
    {
        if (VK_RETURN == pMsg->wParam || VK_ESCAPE == pMsg->wParam)
        {
            return TRUE;
        }
    }
    return CDialogEx::PreTranslateMessage(pMsg);
}
