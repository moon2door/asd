#pragma once

#include "RecognitionObject.h"

namespace SHI
{

	RecognitionObject::RecognitionObject()
	{
	}

	SHI::Recognition::RecogPtr &RecognitionObject::GetRecognition(unsigned int idx)
	{
		return m_estimatePose[idx];
	}

	SHI::Recognition::HookRecogPtr &RecognitionObject::GetRecognitionHook()
	{
		return m_estimateHook;
	}

}