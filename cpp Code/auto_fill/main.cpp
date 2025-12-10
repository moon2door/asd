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
#include <utility>
#include <chrono>
#include <thread>
#include <regex>
#include <cctype>

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
enum class LineEndingStyle { Unknown, LF, CRLF, CR };

struct LineEndingStats {
    size_t crlf = 0;
    size_t lf = 0;
    size_t cr = 0;
};

static LineEndingStats AnalyzeLineEndings(const std::string& text) {
    LineEndingStats stats{};
    const size_t n = text.size();
    size_t i = 0;
    while (i < n) {
        const char c = text[i];
        if (c == '\r') {
            size_t j = i;
            while (j < n && text[j] == '\r') ++j;
            if (j < n && text[j] == '\n') {
                ++stats.crlf;
                ++j;
            }
            else {
                stats.cr += (j - i);
            }
            i = j;
        }
        else if (c == '\n') {
            ++stats.lf;
            ++i;
        }
        else {
            ++i;
        }
    }
    return stats;
}

static LineEndingStyle ChooseLineEndingStyle(const LineEndingStats& stats) {
    const bool hasCRLF = stats.crlf > 0;
    const bool hasLF = stats.lf > 0;
    const bool hasCR = stats.cr > 0;
    const int kinds = (hasCRLF ? 1 : 0) + (hasLF ? 1 : 0) + (hasCR ? 1 : 0);
    if (kinds == 0) return LineEndingStyle::Unknown;
    if (kinds > 1) return LineEndingStyle::Unknown; // mixed endings: keep original bytes
    if (hasCRLF) return LineEndingStyle::CRLF;
    if (hasLF) return LineEndingStyle::LF;
    return LineEndingStyle::CR;
}

static std::string NormalizeToLF(const std::string& text) {
    std::string normalized;
    normalized.reserve(text.size());
    const size_t n = text.size();
    size_t i = 0;
    while (i < n) {
        const char c = text[i];
        if (c == '\r') {
            size_t j = i;
            while (j < n && text[j] == '\r') ++j;
            if (j < n && text[j] == '\n') {
                normalized.push_back('\n');
                i = j + 1;
            }
            else {
                const size_t count = j - i;
                normalized.resize(normalized.size() + count, '\n');
                i = j;
            }
        }
        else if (c == '\n') {
            normalized.push_back('\n');
            ++i;
        }
        else {
            normalized.push_back(c);
            ++i;
        }
    }
    return normalized;
}

static std::string ApplyLineEndingStyle(const std::string& normalizedLF, LineEndingStyle style) {
    if (style == LineEndingStyle::Unknown || style == LineEndingStyle::LF) return normalizedLF;

    std::string converted;
    if (style == LineEndingStyle::CRLF) {
        converted.reserve(normalizedLF.size() + normalizedLF.size() / 2);
        for (char c : normalizedLF) {
            if (c == '\n') converted += "\r\n";
            else converted.push_back(c);
        }
        return converted;
    }
    if (style == LineEndingStyle::CR) {
        converted.reserve(normalizedLF.size());
        for (char c : normalizedLF) {
            if (c == '\n') converted.push_back('\r');
            else converted.push_back(c);
        }
        return converted;
    }
    return normalizedLF;
}

static bool SaveFileUTF8(const std::string& path, const std::string& data_utf8) {
    LineEndingStats stats = AnalyzeLineEndings(data_utf8);
    LineEndingStyle style = ChooseLineEndingStyle(stats);

    const std::string* toWrite = &data_utf8;
    std::string normalized;
    std::string converted;

    if (style != LineEndingStyle::Unknown) {
        normalized = NormalizeToLF(data_utf8);
        if (style == LineEndingStyle::LF) {
            toWrite = &normalized;
        }
        else {
            converted = ApplyLineEndingStyle(normalized, style);
            toWrite = &converted;
        }
    }

    std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
    if (!ofs) return false;
    ofs.write(toWrite->data(), static_cast<std::streamsize>(toWrite->size()));
    return ofs.good();
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

// ---------- Tiny JSON-ish scanners (for partial edit) ----------
static bool IsWS(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// returns end index of JSON string value starting at pos (s[pos]=='"'), or npos on failure
static size_t SkipJsonString(const std::string& s, size_t pos) {
    if (pos >= s.size() || s[pos] != '"') return std::string::npos;
    ++pos;
    bool esc = false;
    for (; pos < s.size(); ++pos) {
        char c = s[pos];
        if (esc) { esc = false; continue; }
        if (c == '\\') { esc = true; continue; }
        if (c == '"') return pos + 1;
    }
    return std::string::npos;
}

// matches balanced {...} starting at pos (s[pos]=='{')
static size_t MatchBrace(const std::string& s, size_t pos) {
    if (pos >= s.size() || s[pos] != '{') return std::string::npos;
    size_t i = pos + 1;
    int depth = 1;
    while (i < s.size()) {
        char c = s[i];
        if (c == '"') {
            size_t ne = SkipJsonString(s, i);
            if (ne == std::string::npos) return std::string::npos;
            i = ne; continue;
        }
        if (c == '{') { ++depth; ++i; continue; }
        if (c == '}') { --depth; ++i; if (depth == 0) return i; continue; }
        ++i;
    }
    return std::string::npos;
}

// matches balanced [...] starting at pos (s[pos]=='[')
static size_t MatchBracket(const std::string& s, size_t pos) {
    if (pos >= s.size() || s[pos] != '[') return std::string::npos;
    size_t i = pos + 1;
    int depth = 1;
    while (i < s.size()) {
        char c = s[i];
        if (c == '"') {
            size_t ne = SkipJsonString(s, i);
            if (ne == std::string::npos) return std::string::npos;
            i = ne; continue;
        }
        if (c == '[') { ++depth; ++i; continue; }
        if (c == ']') { --depth; ++i; if (depth == 0) return i; continue; }
        ++i;
    }
    return std::string::npos;
}

// skip a JSON value (string/number/true/false/null/object/array), return end index (exclusive)
static size_t SkipJsonValue(const std::string& s, size_t pos) {
    if (pos >= s.size()) return std::string::npos;
    char c = s[pos];
    if (c == '"') return SkipJsonString(s, pos);
    if (c == '{')  return MatchBrace(s, pos);
    if (c == '[')  return MatchBracket(s, pos);
    // number / true / false / null / bare token until , } ]
    size_t i = pos;
    while (i < s.size()) {
        char d = s[i];
        if (d == ',' || d == '}' || d == ']' || d == '\n' || d == '\r') break;
        i++;
    }
    return i;
}

// JSON-quote (produce a valid JSON string literal including quotes)
static std::string JsonQuote(const std::string& in) {
    std::string out; out.reserve(in.size() + 16);
    out.push_back('"');
    for (unsigned char c : in) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '\"': out += "\\\""; break;
        case '\b': out += "\\b";  break;
        case '\f': out += "\\f";  break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:
            if (c < 0x20) {
                char buf[7]; _snprintf_s(buf, _TRUNCATE, "\\u%04X", (unsigned)c);
                out += buf;
            }
            else out.push_back((char)c);
        }
    }
    out.push_back('"');
    return out;
}

// find "key" : "string" inside block; returns unescaped value or empty
static std::string FindStringProp(const std::string& block, const std::string& key) {
    std::regex re("\"" + key + R"(\"\s*:\s*\"((?:\\.|[^\"\\])*)\")");
    std::smatch m;
    if (std::regex_search(block, m, re)) {
        std::string v = m[1].str();
        std::string out; out.reserve(v.size());
        bool esc = false;
        for (char ch : v) {
            if (!esc) {
                if (ch == '\\') esc = true;
                else out.push_back(ch);
            }
            else {
        switch (ch) { case 'n': out += '\n'; break; case 'r': out += '\r'; break; case 't': out += '\t'; break; case '\\': out += '\\'; break; case '\"': out += '\"'; break; default: out.push_back(ch); }
                              esc = false;
            }
        }
        return out;
    }
    return {};
}

// replace existing "prop": <value> at top-level of block, or insert if missing.
// newLiteral must be a valid JSON literal (already quoted if string). Minimal diff: no extra spaces/newlines.
static bool ReplaceOrInsertProp(std::string& block, const std::string& prop, const std::string& newLiteral) {
    if (block.size() < 2 || block.front() != '{' || block.back() != '}') return false;

    // -------- replace path (존재하면 값만 교체: 공백/콤마 보존) --------
    size_t i = 1;
    while (i < block.size() - 1) {
        while (i < block.size() - 1 && IsWS(block[i])) ++i;
        if (i >= block.size() - 1 || block[i] == '}') break;

        if (block[i] != '"') break; // 비정상 형식 -> 삽입으로
        size_t key_end = SkipJsonString(block, i);
        if (key_end == std::string::npos) break;

        // key 문자열 언이스케이프 후 비교
        std::string keyClean;
        {
            bool esc = false;
            for (size_t k = i + 1; k < key_end - 1; ++k) {
                char c = block[k];
                if (!esc) { if (c == '\\') esc = true; else keyClean.push_back(c); }
                else {
        switch (block[k]) { case '\"': keyClean.push_back('\"'); break; case '\\': keyClean.push_back('\\'); break; case 'n': keyClean.push_back('\n'); break; default: keyClean.push_back(block[k]); }
                                     esc = false;
                }
            }
        }

        size_t p = key_end;
        while (p < block.size() - 1 && IsWS(block[p])) ++p;
        if (p >= block.size() - 1 || block[p] != ':') { i = p + 1; continue; }
        ++p;
        while (p < block.size() - 1 && IsWS(block[p])) ++p;

        size_t vBeg = p;
        size_t vEnd = SkipJsonValue(block, p);
        if (vEnd == std::string::npos) break;

        if (keyClean == prop) {
            block.replace(vBeg, vEnd - vBeg, newLiteral); // 값만 교체
            return true;
        }

        i = vEnd;
        while (i < block.size() - 1 && IsWS(block[i])) ++i;
        if (i < block.size() - 1 && block[i] == ',') ++i;
    }

    // -------- insert path (공백/개행 없이 } 직전에 삽입) --------
    bool empty = true;
    for (size_t k = 1; k < block.size() - 1; ++k) { if (!IsWS(block[k])) { empty = false; break; } }

    std::string ins;
    if (!empty) ins.push_back(',');
    ins += "\"" + prop + "\":" + newLiteral;

    block.insert(block.size() - 1, ins); // '}' 직전 삽입
    return true;
}

static std::string FormatRotorBlock(const std::string& block) {
    json obj = json::parse(block);
    static const std::vector<std::string> preferredOrder = {
        "altitudeAngles",
        "azimuthAngles",
        "intrinsicParameters",
        "extrinsicParameters",
        "addrLidar",
        "addrDsp",
        "addrBind",
        "prodLine",
        "LidarSerial",
        "RotorSerial",
        "LidarOperationStart",
        "RotorOperationStart",
        "lidarProfile",
        "scanStart"
    };

    std::vector<std::pair<std::string, json>> ordered;
    ordered.reserve(obj.size());

    for (const auto& key : preferredOrder) {
        auto it = obj.find(key);
        if (it != obj.end()) {
            ordered.emplace_back(key, it.value());
            obj.erase(it);
        }
    }
    for (auto it = obj.begin(); it != obj.end(); ++it) {
        ordered.emplace_back(it.key(), it.value());
    }

    const char* newline = (block.find("\r\n") != std::string::npos) ? "\r\n" : "\n";
    const std::string indent = "\t\t";
    std::string result = "{";
    if (!ordered.empty()) {
        result += newline;
        for (size_t idx = 0; idx < ordered.size(); ++idx) {
            result += indent + "\"" + ordered[idx].first + "\":\t" + ordered[idx].second.dump();
            if (idx + 1 < ordered.size()) {
                result += ",";
                result += newline;
            }
            else {
                result += newline;
            }
        }
    }
    else {
        result += newline;
    }
    result += "\t}";
    return result;
}

// find all top-level "Rotor*" : { ... } blocks
struct RotorBlock { std::string key; size_t objBeg; size_t objEnd; }; // [objBeg,objEnd)
static std::vector<RotorBlock> FindRotorBlocks(const std::string& raw) {
    std::vector<RotorBlock> out;
    size_t i = 0;
    int brace = 0;
    while (i < raw.size()) {
        char c = raw[i];
        if (c == '"') {
            size_t e = SkipJsonString(raw, i);
            if (e == std::string::npos) { ++i; continue; }
            if (brace == 1) {
                std::string key;
                { // extract plain text
                    bool esc = false; for (size_t k = i + 1; k < e - 1; ++k) { char ch = raw[k]; if (!esc) { if (ch == '\\') esc = true; else key.push_back(ch); } else { key.push_back(raw[k]); esc = false; } }
                }
                size_t p = e;
                while (p < raw.size() && IsWS(raw[p])) ++p;
                if (p < raw.size() && raw[p] == ':') {
                    ++p; while (p < raw.size() && IsWS(raw[p])) ++p;
                    if (p < raw.size() && raw[p] == '{' && key.rfind("Rotor", 0) == 0) {
                        size_t objBeg = p;
                        size_t objEnd = MatchBrace(raw, p);
                        if (objEnd != std::string::npos) {
                            out.push_back({ key, objBeg, objEnd });
                            i = objEnd; // continue after object
                            continue;
                        }
                    }
                }
            }
            i = e;
            continue;
        }
        if (c == '{') { ++brace; ++i; continue; }
        if (c == '}') { --brace; if (brace < 0) brace = 0; ++i; continue; }
        ++i;
    }
    return out;
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

// ---------- JSON helpers for meta ----------
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

    std::string raw;
    if (!LoadFileUTF8(cfgPath, raw)) {
        printf("[ERROR] Cannot open config file: %s\n", cfgPath.c_str());
        printf("Press Enter to exit..."); getchar(); return 1;
    }

    auto blocks = FindRotorBlocks(raw);
    int total = 0, updated = 0;

    for (size_t idx = 0; idx < blocks.size(); ++idx) {
        auto& blk = blocks[idx];
        total++;

        std::string block = raw.substr(blk.objBeg, blk.objEnd - blk.objBeg);

        // addrLidar 찾기 (없으면 스킵: 원본 유지)
        std::string ip = FindStringProp(block, "addrLidar");
        if (ip.empty()) {
            printf("[WARN] %s: addrLidar is empty, skipped.\n", blk.key.c_str());
            continue;
        }

        std::string url = "http://" + ip + "/api/v1/sensor/metadata";
        printf("[INFO] %s: GET %s\n", blk.key.c_str(), url.c_str());

        HttpResult hr = HttpGet(Utf8ToWide(url));
        if (!(hr.ok && hr.status >= 200 && hr.status < 300)) {
            printf("[ERROR] %s: request failed: %s (HTTP %lu)\n", blk.key.c_str(), hr.err.c_str(), hr.status);
            continue; // 이 Rotor는 스킵
        }

        json meta;
        try { meta = json::parse(hr.body); }
        catch (const std::exception& e) {
            printf("[ERROR] %s: failed to parse response JSON: %s\n", blk.key.c_str(), e.what());
            continue;
        }

        std::string prodLine = JGetStr(meta, "sensor_info", "prod_line");
        std::string profile = JGetStr(meta, "lidar_data_format", "udp_profile_lidar");
        if (profile.empty()) profile = JGetStr(meta, "config_params", "udp_profile_lidar");

        json alt = JGetArr(meta, "beam_intrinsics", "beam_altitude_angles");
        json azi = JGetArr(meta, "beam_intrinsics", "beam_azimuth_angles");

        bool changed = false;
        if (!prodLine.empty()) changed |= ReplaceOrInsertProp(block, "prodLine", JsonQuote(prodLine));
        if (!profile.empty())  changed |= ReplaceOrInsertProp(block, "lidarProfile", JsonQuote(profile));
        if (!alt.empty()) {
            std::string altStr = alt.dump(-1, ' ', false, json::error_handler_t::ignore); // compact "[...]"
            changed |= ReplaceOrInsertProp(block, "altitudeAngles", JsonQuote(altStr));
        }
        if (!azi.empty()) {
            std::string aziStr = azi.dump(-1, ' ', false, json::error_handler_t::ignore);
            changed |= ReplaceOrInsertProp(block, "azimuthAngles", JsonQuote(aziStr));
        }

        if (changed) {
            block = FormatRotorBlock(block);
            const size_t oldLen = blk.objEnd - blk.objBeg;
            raw.replace(blk.objBeg, oldLen, block);
            const ptrdiff_t delta = (ptrdiff_t)block.size() - (ptrdiff_t)oldLen;

            // 뒤 블록 위치 보정 (재스캔 없이)
            for (size_t j = idx + 1; j < blocks.size(); ++j) {
                blocks[j].objBeg += delta;
                blocks[j].objEnd += delta;
            }
            // 현재 블록 길이 갱신
            blk.objEnd = blk.objBeg + block.size();
            updated++;
        }

        printf("[OK] %s: prodLine=\"%s\", lidarProfile=\"%s\"%s%s\n",
            blk.key.c_str(),
            prodLine.c_str(),
            profile.c_str(),
            alt.empty() ? "" : ", altitudeAngles updated",
            azi.empty() ? "" : ", azimuthAngles updated");
    }

    if (!SaveFileUTF8(cfgPath, raw)) {
        printf("[ERROR] Failed to save config: %s\n", cfgPath.c_str());
    }
    else {
        printf("[INFO] Save complete: %s (Rotor %d, updated %d)\n", cfgPath.c_str(), total, updated);
    }

    printf("\nPress Enter to exit...");
    getchar();
    return 0;
}
