
#pragma once
#include <Utility/Types.h>
#include <Utility/CraneObject.h>
#include <Utility/RecognitionObject.h>

#include <Interface/XYZPoint/Proxy/ProxyXYZPoint.h>
#include <Interface/Cluster/Stub/StubCluster.h>
#include <Interface/CraneAttitude/Stub/StubCraneAttitude.h>
#include <Interface/DBInfo/Proxy/ProxyDBInfo.h>
#include <Interface/DBInfo/Stub/StubDBInfo.h>
#include <Interface/PLCInfo/Proxy/ProxyPlcInfo.h>
#include <Recognition/EstimateLLCAzimuth.h>
#include <Recognition/EstimateLLCJib.h>
#include <Recognition/EstimateGCRoom.h>
#include <Recognition/EstimateHookCluster.h>
#include <Recognition/EstimatorVisualizer.h>
#include <Routine/include/Visualization/CImageViewer.h>
#include <Routine/include/Base/CThread.h>
#include <Routine/include/Base/CTimer.h>
#include <queue>
#include <string>

class CDebugObject
{
public:
	CDebugObject() : m_bDebug(false) {}
	void SetDebugMode(bool bDebug) { m_bDebug = bDebug; }
	inline bool IsDebugMode() { return m_bDebug; }
	inline SHI::Object::CSHMStub<SHI::Data::StCluster>* GetStubDebugCluster() { return m_stubDebugCluster.GetStubCluster(); }
private:
	bool m_bDebug;
	SHI::Interface::Cluster::Stub::CStubCluster m_stubDebugCluster;
};

class CClusterProcessor 
	: public SHI::CCraneObject
	, public SHI::RecognitionObject
	, public SHI::Interface::XYZPoint::Proxy::ProxyXYZPoint
	, public SHI::Interface::Cluster::Stub::CStubCluster
	, public SHI::Interface::DBInfo::Proxy::CProxyDBInfo
	, public SHI::Interface::DBInfo::Stub::CStubDBInfo
	, public SHI::Interface::PlcInfo::Proxy::CProxyPlcInfo
	, public SHI::Interface::CraneAttitude::Stub::CStubCraneAttitude
	, public SHI::CEstimatorVisualizer
	, public CDebugObject
{
public :
	CClusterProcessor();
	~CClusterProcessor();
#include "iostream"

class RefTest
{
public:
    void Call()
    {
        std::cout << "Call RefTest" << std::endl;
    }
};

class Test
{
public:
    static int main()
    {
        RefTest rTest;
        rTest.Call();

        std::cout << "Success" << std::endl;
        return 0;
    }
};

	virtual void OnXYZPoint(SHI::Data::StXYZPoints* pData);

	virtual void OnDBInfo(SHI::Data::StDBInfo* pData);

	virtual void OnPlcInfo(SHI::Data::StPlcInfo* pData);

	// Process functions
	bool ProcessInitialize();
	
	bool ProcessPreProcessing(SHI::PointCloud& dst, SHI::Data::StXYZPoints* src);

	bool ProcessCraneAttitude(SHI::PointCloudPtr cloud);

	bool ProcessRemoveCranePoints(SHI::PointCloud& dst, SHI::PointCloud& dstOutlier, SHI::PointCloud& dstExceptionRoi, SHI::PointCloudPtr cloudInput);

	void ProcessPreClustering(SHI::PointCloud& dst, SHI::PointCloud& dstGround, SHI::PointCloudPtr src);

	bool ProcessClustering(SHI::PCLIndicesVector& dstIndices, SHI::PointCloud& dstCloud, SHI::PointCloudPtr& cloudInput);

	bool ProcessLabelCluster(SHI::PCLIndicesVector& dstIndices, SHI::PointCloud& dstCloud, std::vector<unsigned char> &labels, SHI::PointCloudPtr& cloud, const SHI::PCLIndicesVectorPtr& srcIndices, const SHI::PointCloudPtr& cloudBody, const SHI::PointCloudPtr& cloudGround, const SHI::PointCloudPtr& cloudException);

	void ProcessPostProcessing(SHI::Data::StCluster &dst, SHI::PointCloudPtr cloud, SHI::PCLIndicesVectorPtr vIndices, std::vector<unsigned char>& labels, const SHI::CraneAttitudePtr& attitude);

	void ProcessAccumCloudModel(const SHI::PointCloudPtr cloud);
	
private:
	// 입력 데이터 누적
	std::queue<SHI::PointCloudPtr> m_qCloudInput;
    int m_maxAccumSize;

    // DBInfo periodic logger (independent logging cadence)
    Routine::CTimer m_timerDbInfoLog;
    void OnTimerDbInfoLog();
    std::string m_dbInfoCsvFile;
    bool m_dbInfoCsvHeaderWritten = false;
}; 
