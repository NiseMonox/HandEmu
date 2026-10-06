#include "capture.h"

#include <algorithm>
#include <cctype>
#include <cstring>

#include "gestures.h"
#include "vrmath.h"

namespace handemu
{

namespace
{

std::string ToUtf8( const std::wstring &w )
{
	if ( w.empty() )
		return std::string();
	int n = WideCharToMultiByte( CP_UTF8, 0, w.c_str(), (int)w.size(), nullptr, 0, nullptr, nullptr );
	std::string s( n, '\0' );
	WideCharToMultiByte( CP_UTF8, 0, w.c_str(), (int)w.size(), &s[ 0 ], n, nullptr, nullptr );
	return s;
}

bool ReadSmallFile( const std::wstring &path, std::string &out )
{
	HANDLE h = CreateFileW( path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, 0, nullptr );
	if ( h == INVALID_HANDLE_VALUE )
		return false;
	char buf[ 8192 ];
	DWORD read = 0;
	BOOL ok = ReadFile( h, buf, sizeof( buf ) - 1, &read, nullptr );
	CloseHandle( h );
	if ( !ok )
		return false;
	out.assign( buf, read );
	return true;
}

std::string Trim( const std::string &s )
{
	size_t a = 0, b = s.size();
	while ( a < b && std::isspace( (unsigned char)s[ a ] ) )
		a++;
	while ( b > a && std::isspace( (unsigned char)s[ b - 1 ] ) )
		b--;
	return s.substr( a, b - a );
}

} // namespace

void DebugFolder::Init( const std::wstring &folder )
{
	m_folder = folder;
	CreateDirectoryW( folder.c_str(), nullptr );
}

std::string DebugFolder::PathUtf8() const { return ToUtf8( m_folder ); }

std::string DebugFolder::PollCapture( double now )
{
	if ( m_folder.empty() )
		return std::string();

	if ( m_capturing.load() && now >= m_until )
	{
		uint32_t lines;
		{
			std::lock_guard<std::mutex> lock( m_mutex );
			m_capturing = false;
			if ( m_file )
				fclose( m_file );
			m_file = nullptr;
			lines = m_lines;
		}
		MoveFileExW( m_tmpPath.c_str(), m_finalPath.c_str(), MOVEFILE_REPLACE_EXISTING );
		return "Capture '" + m_label + "' saved (" + std::to_string( lines ) + " lines): " + ToUtf8( m_finalPath );
	}

	if ( m_capturing.load() || now < m_nextRequestCheck )
		return std::string();
	m_nextRequestCheck = now + 0.25;

	const std::wstring request = m_folder + L"\\request.txt";
	if ( GetFileAttributesW( request.c_str() ) == INVALID_FILE_ATTRIBUTES )
		return std::string();
	std::string text;
	ReadSmallFile( request, text );
	DeleteFileW( request.c_str() );

	// "label [seconds]"
	std::string label;
	double seconds = 3.0;
	{
		std::string t = Trim( text );
		size_t space = t.find_first_of( " \t\r\n" );
		std::string first = t.substr( 0, space );
		for ( char c : first )
		{
			if ( std::isalnum( (unsigned char)c ) || c == '_' || c == '-' )
				label += c;
			if ( label.size() >= 40 )
				break;
		}
		if ( space != std::string::npos )
		{
			double s = atof( t.c_str() + space );
			if ( s > 0 )
				seconds = s;
		}
	}
	if ( label.empty() )
		label = "capture";
	seconds = std::clamp( seconds, 0.2, 30.0 );

	std::wstring wlabel( label.begin(), label.end() );
	m_tmpPath = m_folder + L"\\" + wlabel + L".tmp";
	m_finalPath = m_folder + L"\\" + wlabel + L".csv";
	FILE *f = nullptr;
	if ( _wfopen_s( &f, m_tmpPath.c_str(), L"wb" ) != 0 || !f )
		return "Capture '" + label + "' could not create " + ToUtf8( m_tmpPath );
	setvbuf( f, nullptr, _IOFBF, 1 << 20 );
	fprintf( f, "# HandEmu skeleton capture '%s', %.1f s\n", label.c_str(), seconds );
	fprintf( f, "# S,t,side,range,31 x (px,py,pz,qw,qx,qy,qz) = from the source driver; O,... = sent by HandEmu (range -1 = both)\n" );
	fprintf( f, "# P,t,side,px,py,pz,qw,qx,qy,qz,valid = source device pose in world space\n" );
	{
		std::lock_guard<std::mutex> lock( m_mutex );
		m_file = f;
		m_lines = 0;
		m_label = label;
		m_start = now;
		m_until = now + seconds;
		m_capturing = true;
	}
	char msg[ 160 ];
	snprintf( msg, sizeof( msg ), "Capture '%s' started (%.1f s)", label.c_str(), seconds );
	return msg;
}

void DebugFolder::WriteLine( const char *line, size_t length )
{
	std::lock_guard<std::mutex> lock( m_mutex );
	if ( !m_file )
		return;
	fwrite( line, 1, length, m_file );
	m_lines++;
}

void DebugFolder::RecordSkeleton( char tag, int side, int range, const vr::VRBoneTransform_t *bones, double now )
{
	if ( !Capturing() )
		return;
	char line[ 8192 ];
	int n = snprintf( line, sizeof( line ), "%c,%.4f,%d,%d", tag, now - m_start, side, range );
	for ( uint32_t i = 0; i < kBoneCount && n > 0 && n < (int)sizeof( line ) - 128; i++ )
	{
		const vr::VRBoneTransform_t &b = bones[ i ];
		n += snprintf( line + n, sizeof( line ) - n, ",%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f", b.position.v[ 0 ], b.position.v[ 1 ], b.position.v[ 2 ],
			b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z );
	}
	if ( n <= 0 || n >= (int)sizeof( line ) - 2 )
		return;
	line[ n++ ] = '\n';
	WriteLine( line, n );
}

void DebugFolder::RecordPose( int side, const vr::DriverPose_t &pose, double now )
{
	if ( !Capturing() )
		return;
	Quat q = FromVR( pose.qWorldFromDriverRotation ) * FromVR( pose.qRotation );
	Vec3 p = Rotate( FromVR( pose.qWorldFromDriverRotation ), FromVR( pose.vecPosition ) ) + FromVR( pose.vecWorldFromDriverTranslation );
	char line[ 256 ];
	int n = snprintf( line, sizeof( line ), "P,%.4f,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d\n", now - m_start, side, p.x, p.y, p.z, q.w, q.x, q.y, q.z,
		pose.poseIsValid ? 1 : 0 );
	if ( n > 0 && n < (int)sizeof( line ) )
		WriteLine( line, n );
}

bool DebugFolder::PollLiveFile( std::vector<std::pair<std::string, std::string>> &entries )
{
	if ( m_folder.empty() )
		return false;
	const std::wstring path = m_folder + L"\\live.txt";
	WIN32_FILE_ATTRIBUTE_DATA data;
	if ( !GetFileAttributesExW( path.c_str(), GetFileExInfoStandard, &data ) )
	{
		bool changed = m_liveSeen; // deleted: back to the defaults
		m_liveSeen = false;
		entries.clear();
		return changed;
	}
	if ( m_liveSeen && CompareFileTime( &data.ftLastWriteTime, &m_liveTime ) == 0 )
		return false;
	std::string text;
	if ( !ReadSmallFile( path, text ) )
		return false; // still being written; try again next time
	m_liveSeen = true;
	m_liveTime = data.ftLastWriteTime;

	entries.clear();
	size_t pos = 0;
	while ( pos < text.size() )
	{
		size_t end = text.find( '\n', pos );
		if ( end == std::string::npos )
			end = text.size();
		std::string line = text.substr( pos, end - pos );
		pos = end + 1;
		size_t hash = line.find( '#' );
		if ( hash != std::string::npos )
			line.erase( hash );
		line = Trim( line );
		if ( line.empty() )
			continue;
		size_t sep = line.find_first_of( "= \t:" );
		if ( sep == std::string::npos )
			continue;
		std::string key = Trim( line.substr( 0, sep ) );
		std::string value = line.substr( sep + 1 );
		size_t eq = value.find_first_not_of( "= \t:" );
		value = eq == std::string::npos ? std::string() : Trim( value.substr( eq ) );
		if ( !key.empty() )
			entries.emplace_back( key, value );
	}
	return true;
}

} // namespace handemu
