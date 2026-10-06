// Diagnostics folder next to the installed driver (<driver>\debug):
//   request.txt  - "label [seconds]": records the raw source skeletons (and what HandEmu sends) for
//                  the next few seconds into <label>.csv, then deletes the request.
//   live.txt     - "key = value" lines re-read whenever the file changes, so skeleton tuning can be
//                  adjusted while SteamVR is running.
#pragma once

#include <windows.h>

#include <atomic>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <openvr_driver.h>

namespace handemu
{

class DebugFolder
{
public:
	void Init( const std::wstring &folder );
	std::string PathUtf8() const;

	// Starts a capture when request.txt appears and finishes it when its time is up.
	// Returns a line worth logging, or an empty string.
	std::string PollCapture( double now );

	// True when live.txt appeared or changed since the last call; entries = its key/value pairs.
	bool PollLiveFile( std::vector<std::pair<std::string, std::string>> &entries );

	bool Capturing() const { return m_capturing.load(); }
	// tag: 'S' = from the source driver, 'O' = sent by HandEmu.
	void RecordSkeleton( char tag, int side, int range, const vr::VRBoneTransform_t *bones, double now );
	void RecordPose( int side, const vr::DriverPose_t &pose, double now );

private:
	void WriteLine( const char *line, size_t length );

	std::wstring m_folder;
	std::mutex m_mutex;
	FILE *m_file = nullptr;
	std::atomic<bool> m_capturing{ false };
	double m_start = 0;
	double m_until = 0;
	uint32_t m_lines = 0;
	std::string m_label;
	std::wstring m_tmpPath;
	std::wstring m_finalPath;
	double m_nextRequestCheck = 0;

	bool m_liveSeen = false;
	FILETIME m_liveTime{};
};

} // namespace handemu
