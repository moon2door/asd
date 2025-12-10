#include "Config.h"
#include <string>

namespace Utility
{

	void SetConfigVector(const std::string& groupName, const std::string& identityName, const std::vector<double_t> &value, const std::string& fileName /*= Routine::GetMakeConfigName()*/)
	{
		std::string str;
		if (!value.empty())
		{
			str += "[";
			for (const auto val : value)
			{
				char tmp[256] = "";
				sprintf_s(tmp, "%lf,", val);  // NOLINT(cert-err33-c)
				str += tmp;
			}
			*(str.end()-1) = ']';
		}

		Routine::SetConfigString(groupName, identityName, str, fileName);
	}

	void SetConfigVector(const std::string& groupName, const std::string& identityName, const std::vector<float_t> &value, const std::string& fileName /*= Routine::GetMakeConfigName()*/)
	{
		std::vector<double_t> tmp;
		tmp.reserve(value.size());
		for (const auto& val : value)
		{
			tmp.emplace_back(static_cast<double_t>(val));
		}
		return SetConfigVector(groupName, identityName, tmp, fileName);
	}

	void GetConfigVector(const std::string& groupName, const std::string& identityName, std::vector<double_t> &dst, const std::string& fileName /*= Routine::GetMakeConfigName()*/)
	{
		const std::string str = Routine::GetConfigString(groupName, identityName, "[]", fileName);
		const std::string numbers("0123456789.-");
		dst.clear();

		size_t pos = 0;
		size_t s = str.find_first_of(numbers, pos);
		size_t e = str.find_first_not_of(numbers, s);
		while (s != std::string::npos && e != std::string::npos)
		{
			if (s != std::string::npos && e != std::string::npos)
			{
				std::string sub = str.substr(s, e - s);

				double_t val = 0;
				if (sscanf_s(sub.c_str(), "%lf,", &val))
				{
					dst.push_back(val);
				}
			}

			pos = e;
			s = str.find_first_of(numbers, pos);
			e = str.find_first_not_of(numbers, s);
		}
	}

	void GetConfigVector(const std::string& groupName, const std::string& identityName, std::vector<float_t> &dst, const std::string& fileName /*= Routine::GetMakeConfigName()*/)
	{
		std::vector<double_t> numbers;
		GetConfigVector(groupName, identityName, numbers, fileName);

		for (const auto& num : numbers)
		{
			dst.push_back(static_cast<float_t>(num));
		}
	}

	Routine::CMatrix33f GetConfigMatrix33f(const std::string& groupName, const std::string& identityName, const std::string& fileName)
	{
		Routine::CMatrix33f matrix = Routine::CMatrix33f::Identity();
		const std::string str = Routine::GetConfigString(groupName, identityName, "[1 0 0 0 1 0 0 0 1]", fileName);
		const std::string numbers("0123456789.-");

		std::vector<float_t> params;
		size_t pos = 0;
		size_t s = str.find_first_of(numbers, pos);
		size_t e = str.find_first_not_of(numbers, s);
		while (s != std::string::npos && e != std::string::npos)
		{
			if (s != std::string::npos && e != std::string::npos)
			{
				std::string sub = str.substr(s, e - s);

				float_t val = 0;
				if (sscanf_s(sub.c_str(), "%f", &val))
				{
					params.push_back(val);
				}
			}

			pos = e;
			s = str.find_first_of(numbers, pos);
			e = str.find_first_not_of(numbers, s);
		}

		if(params.size() == static_cast<uint64_t>(matrix.size()))
		{
			for (int64_t i = 0; i < matrix.size(); i++)
			{
				// ReSharper disable once CppRedundantParentheses
				const int64_t idx = (i % 3) * 3 + (i / 3);
				matrix(idx) = params[i];
			}
		}
		return matrix;
	}

	Routine::CMatrix44f GetConfigMatrix44f(const std::string& groupName, const std::string& identityName,
		const std::string& fileName)
	{
		Routine::CMatrix44f matrix = Routine::CMatrix44f::Identity();
		const std::string str = Routine::GetConfigString(groupName, identityName, "[1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1]", fileName);
		const std::string numbers("0123456789.-");

		std::vector<float_t> params;
		size_t pos = 0;
		size_t s = str.find_first_of(numbers, pos);
		size_t e = str.find_first_not_of(numbers, s);
		while (s != std::string::npos && e != std::string::npos)
		{
			if (s != std::string::npos && e != std::string::npos)
			{
				std::string sub = str.substr(s, e - s);

				float_t val = 0;
				if (sscanf_s(sub.c_str(), "%f", &val))
				{
					params.push_back(val);
				}
			}

			pos = e;
			s = str.find_first_of(numbers, pos);
			e = str.find_first_not_of(numbers, s);
		}

		if (params.size() == static_cast<uint64_t>(matrix.size()))
		{
			for (int64_t i = 0; i < matrix.size(); i++)
			{
				// ReSharper disable once CppRedundantParentheses
				const int64_t idx = (i % 4) * 4 + (i / 4);
				matrix(idx) = params[i];
			}
		}
		return matrix;
	}

	void SetConfigMatrix44f(const std::string& groupName, const std::string& identityName,
		const Routine::CMatrix44f& matrix, const std::string& fileName)
	{
		std::vector<float_t> value;
		value.reserve(matrix.size());
		for(int64_t i=0; i< matrix.size(); i++)
		{
			// ReSharper disable once CppRedundantParentheses
			const int64_t idx = (i % 4) * 4 + (i / 4);
			value.emplace_back(matrix(idx));
		}
		SetConfigVector(groupName, identityName, value, fileName);
	}

	void SetConfigMatrix33f(const std::string& groupName, const std::string& identityName,
		const Routine::CMatrix33f& matrix, const std::string& fileName)
	{
		std::vector<float_t> value;
		value.reserve(matrix.size());
		for (int64_t i = 0; i < matrix.size(); i++)
		{
			// ReSharper disable once CppRedundantParentheses
			const int64_t idx = (i % 3) * 3 + (i / 3);
			value.emplace_back(matrix(idx));
		}
		SetConfigVector(groupName, identityName, value, fileName);
	}
}
