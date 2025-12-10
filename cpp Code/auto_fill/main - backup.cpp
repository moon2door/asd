// main.cpp — InterfaceRotor metadata auto-fill (Win7~11, portable)
// Build: MSVC, Win32(x86), Release, Runtime Library: /MT
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winhttp.h>
#pragma comment(lib, "winhttp.lib")

#include <cstdio>
#include <cstdlib>
#include <string>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>

#include "json.hpp" // nlohmann json (single header)
using json = nlohmann::ordered_json;

#ifndef WINHTTP_FLAG_SECURE_PROTOCOL_TLS1_1
#define WINHTTP_FLAG_SECURE_PROTOCOL_TLS1_1 0x00000200
#endif
#ifndef WINHTTP_FLAG_SECURE_PROTOCOL_TLS1_2
#define WINHTTP_FLAG_SECURE_PROTOCOL_TLS1_2 0x00000800
#endif

// ---------- Helpers ----------
static std::wstring Utf8ToWide(const std::string& s) {
    if (s.empty()) return L"";
    int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), nullptr, 0);
    std::wstring w(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), &w[0], n);
    return w;
}
static std::string WideToUtf8(const std::wstring& w) {
    if (w.empty()) return "";
    int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), (int)w.size(), nullptr, 0, nullptr, nullptr);
    std::string s(n, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), (int)w.size(), &s[0], n, nullptr, nullptr);
    return s;
}
static bool LoadFileUTF8(const std::string& path, std::string& out) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    out.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    return true;
}
static bool SaveFileUTF8(const std::string& path, const std::string& data_utf8_lf) {
    // LF -> CRLF
    std::string crlf;
    crlf.reserve(data_utf8_lf.size() + 16);
    for (char c : data_utf8_lf) {
        if (c == '\n') crlf += "\r\n";
        else crlf += c;
    }
    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs) return false;
    // no BOM
    ofs.write(crlf.data(), (std::streamsize)crlf.size());
    return true;
}

static std::string WinErrMsg(DWORD e) {
    wchar_t* wmsg = nullptr;
    DWORD len = FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, e, 0, (LPWSTR)&wmsg, 0, nullptr);
    std::string s;
    if (len && wmsg) { s = WideToUtf8(wmsg); LocalFree(wmsg); }
    char buf[64]; _snprintf_s(buf, _TRUNCATE, " (err=%lu)", e);
    s += buf;
    return s;
}

// ---------- HTTP ----------
struct HttpResult {
    bool ok = false;
    DWORD status = 0;
    std::string body;
    std::string err;
};

static HttpResult HttpGet(const std::wstring& url_w, int timeout_ms = 10000) {
    HttpResult r;

    URL_COMPONENTS uc{};
    uc.dwStructSize = sizeof(uc);
    wchar_t host[256]{}, path[2048]{};
    uc.lpszHostName = host;  uc.dwHostNameLength = _countof(host);
    uc.lpszUrlPath = path;  uc.dwUrlPathLength = _countof(path);
    if (!WinHttpCrackUrl(url_w.c_str(), 0, 0, &uc)) {
        r.err = "WinHttpCrackUrl failed" + WinErrMsg(GetLastError());
        return r;
    }

    HINTERNET hSession = WinHttpOpen(L"InterfaceRotorUpdater/1.0",
        WINHTTP_ACCESS_TYPE_NO_PROXY, WINHTTP_NO_PROXY_NAME, WINHTTP_NO_PROXY_BYPASS, 0);
    if (!hSession) { r.err = "WinHttpOpen failed" + WinErrMsg(GetLastError()); return r; }

    WinHttpSetTimeouts(hSession, timeout_ms, timeout_ms, timeout_ms, timeout_ms);

    HINTERNET hConnect = WinHttpConnect(hSession, host, uc.nPort, 0);
    if (!hConnect) { r.err = "WinHttpConnect failed" + WinErrMsg(GetLastError()); WinHttpCloseHandle(hSession); return r; }

    DWORD flags = (uc.nScheme == INTERNET_SCHEME_HTTPS) ? WINHTTP_FLAG_SECURE : 0;
    HINTERNET hRequest = WinHttpOpenRequest(hConnect, L"GET", path, nullptr,
        WINHTTP_NO_REFERER, WINHTTP_DEFAULT_ACCEPT_TYPES, flags);
    if (!hRequest) { r.err = "WinHttpOpenRequest failed" + WinErrMsg(GetLastError()); WinHttpCloseHandle(hConnect); WinHttpCloseHandle(hSession); return r; }

    const wchar_t* hdr = L"Accept: application/json\r\nConnection: close\r\n";
    WinHttpAddRequestHeaders(hRequest, hdr, -1L, WINHTTP_ADDREQ_FLAG_ADD);

    if (!WinHttpSendRequest(hRequest, WINHTTP_NO_ADDITIONAL_HEADERS, 0, nullptr, 0, 0, 0)) {
        r.err = "WinHttpSendRequest failed" + WinErrMsg(GetLastError());
    }
    else if (!WinHttpReceiveResponse(hRequest, nullptr)) {
        r.err = "WinHttpReceiveResponse failed" + WinErrMsg(GetLastError());
    }
    else {
        DWORD status = 0; DWORD stSize = sizeof(status);
        if (WinHttpQueryHeaders(hRequest,
            WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
            WINHTTP_HEADER_NAME_BY_INDEX, &status, &stSize, WINHTTP_NO_HEADER_INDEX)) {
            r.status = status;
        }
        std::vector<char> buf;
        for (;;) {
            DWORD avail = 0;
            if (!WinHttpQueryDataAvailable(hRequest, &avail)) break;
            if (avail == 0) { r.ok = (r.status >= 200 && r.status < 300); break; }
            size_t off = buf.size(); buf.resize(off + avail);
            DWORD read = 0;
            if (!WinHttpReadData(hRequest, buf.data() + off, avail, &read)) break;
            buf.resize(off + read);
        }
        r.body.assign(buf.begin(), buf.end());
    }

    WinHttpCloseHandle(hRequest);
    WinHttpCloseHandle(hConnect);
    WinHttpCloseHandle(hSession);
    return r;
}

// ---------- JSON helpers ----------
static std::string JGetStr(const json& j, const char* a, const char* b) {
    if (j.contains(a) && j[a].contains(b) && j[a][b].is_string()) return j[a][b].get<std::string>();
    return {};
}
static json JGetArr(const json& j, const char* a, const char* b) {
    if (j.contains(a) && j[a].contains(b) && j[a][b].is_array()) return j[a][b];
    return json::array();
}

// ---------- Main ----------
int wmain(int argc, wchar_t* argv[]) {
    SetConsoleOutputCP(CP_UTF8);

    std::string cfgPath = "interfacerotor.json";
    if (argc >= 2) cfgPath = WideToUtf8(argv[1]);

    printf("[INFO] config: %s\n", cfgPath.c_str());

    std::string text;
    if (!LoadFileUTF8(cfgPath, text)) {
        printf("[ERROR] Cannot open config file: %s\n", cfgPath.c_str());
        printf("Press Enter to exit..."); getchar(); return 1;
    }

    json cfg;
    try { cfg = json::parse(text); }
    catch (const std::exception& e) {
        printf("[ERROR] Failed to parse config JSON: %s\n", e.what());
        printf("Press Enter to exit..."); getchar(); return 1;
    }

    int total = 0, updated = 0;

    for (auto it = cfg.begin(); it != cfg.end(); ++it) {
        const std::string& key = it.key();
        if (key.rfind("Rotor", 0) != 0 || !it.value().is_object()) continue;

        total++;
        json& rj = it.value();
        std::string ip = rj.value("addrLidar", "");
        if (ip.empty()) {
            printf("[WARN] %s: addrLidar is empty, skipped.\n", key.c_str());
            continue;
        }

        std::string url = "http://" + ip + "/api/v1/sensor/metadata";
        printf("[INFO] %s: GET %s\n", key.c_str(), url.c_str());

        HttpResult hr = HttpGet(Utf8ToWide(url));
        if (!(hr.ok && hr.status >= 200 && hr.status < 300)) {
            printf("[ERROR] %s: request failed: %s\n", key.c_str(), hr.err.c_str());
            continue;
        }

        json meta;
        try { meta = json::parse(hr.body); }
        catch (const std::exception& e) {
            printf("[ERROR] %s: failed to parse response JSON: %s\n", key.c_str(), e.what());
            continue;
        }

        std::string prodLine = JGetStr(meta, "sensor_info", "prod_line");
        std::string profile = JGetStr(meta, "lidar_data_format", "udp_profile_lidar");
        if (profile.empty()) profile = JGetStr(meta, "config_params", "udp_profile_lidar");

        json alt = JGetArr(meta, "beam_intrinsics", "beam_altitude_angles");
        json azi = JGetArr(meta, "beam_intrinsics", "beam_azimuth_angles");

        if (!prodLine.empty())  rj["prodLine"] = prodLine;
        if (!profile.empty())   rj["lidarProfile"] = profile;
        if (!alt.empty())       rj["altitudeAngles"] = alt.dump(-1, ' ', false, json::error_handler_t::ignore);
        if (!azi.empty())       rj["azimuthAngles"] = azi.dump(-1, ' ', false, json::error_handler_t::ignore);

        updated++;
        printf("[OK] %s: prodLine=\"%s\", lidarProfile=\"%s\", angles updated.\n",
            key.c_str(), prodLine.c_str(), profile.c_str());
    }

    if (!SaveFileUTF8(cfgPath, cfg.dump(2))) {
        printf("[ERROR] Failed to save config: %s\n", cfgPath.c_str());
    }
    else {
        printf("[INFO] Save complete: %s (Rotor %d, updated %d)\n", cfgPath.c_str(), total, updated);
    }

    printf("\nPress Enter to exit...");
    getchar();
    return 0;
}
