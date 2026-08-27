#include "CFile.hpp"
#include <sys/sysinfo.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <openssl/sha.h>
#include <algorithm>
#include <iostream>
#include <regex>

namespace {

std::ios::openmode ToOpenMode(const char* mode)
{
    std::ios::openmode openMode = std::ios::binary;
    if (!mode)
    {
        return openMode;
    }

    const std::string modeStr(mode);

    // Parse common fopen-style modes.
    if (modeStr.find('r') != std::string::npos)
    {
        openMode |= std::ios::in;
    }
    if (modeStr.find('w') != std::string::npos)
    {
        openMode |= std::ios::out | std::ios::trunc;
    }
    if (modeStr.find('a') != std::string::npos)
    {
        openMode |= std::ios::out | std::ios::app;
    }
    if (modeStr.find('+') != std::string::npos)
    {
        openMode |= std::ios::in | std::ios::out;
    }

    // If no direction was specified, default to read+write so the class can
    // both read and write the file (matching the original FILE* behaviour).
    if (!(openMode & (std::ios::in | std::ios::out)))
    {
        openMode |= std::ios::in | std::ios::out;
    }

    return openMode;
}

// Flags used solely to obtain a raw fd for flock()'ing the file; this fd is
// never used for I/O, so it just needs whatever access is enough to open it
// (creating it if the caller's mode implies the file may not exist yet).
int ToLockOpenFlags(const char* mode)
{
    if (!mode)
    {
        return O_RDWR | O_CREAT;
    }

    const std::string modeStr(mode);
    const bool readOnly = modeStr.find('r') != std::string::npos &&
                           modeStr.find('w') == std::string::npos &&
                           modeStr.find('a') == std::string::npos &&
                           modeStr.find('+') == std::string::npos;

    return readOnly ? O_RDONLY : (O_RDWR | O_CREAT);
}

} // namespace

CFile::CFile(const char* filePath, const char* mode)
{
    if (filePath)
    {
        open(filePath, mode);
    }
}

CFile::~CFile()
{
    close();
}

int CFile::open(const char* filePath, const char* mode)
{
    if (m_FileStream.is_open())
    {
        // Already open on this instance; caller must close() first rather
        // than silently losing track of the previous file.
        return -1;
    }

    if (!filePath)
    {
        return -1;
    }

    std::filesystem::path path(filePath);
    if (path.has_parent_path() && !std::filesystem::exists(path.parent_path()))
    {
        std::filesystem::create_directories(path.parent_path());
    }

    // Take an exclusive, non-blocking advisory lock on the file so no other
    // process (or another CFile instance) can hold it open at the same
    // time. This fd is only used for locking, not for I/O.
    int lockFd = ::open(filePath, ToLockOpenFlags(mode), 0644);
    if (lockFd < 0)
    {
        return -1;
    }

    if (flock(lockFd, LOCK_EX | LOCK_NB) < 0)
    {
        // Someone else already has this file open.
        ::close(lockFd);
        return -1;
    }

    auto openMode = ToOpenMode(mode);
    m_FileStream.open(filePath, openMode);
    if (!m_FileStream.is_open())
    {
        flock(lockFd, LOCK_UN);
        ::close(lockFd);
        return -1;
    }

    m_LockFd = lockFd;
    internalFilePath = filePath;

    // Determine current file size.
    m_FileStream.seekg(0, std::ios::end);
    std::streampos fileSize = m_FileStream.tellg();
    m_FileStream.seekg(0, std::ios::beg);
    m_Size = (fileSize > 0) ? static_cast<size_t>(fileSize) : 0;
    m_Offset = 0;

    return 0;
}

void CFile::close()
{
    if (m_FileStream.is_open())
    {
        m_FileStream.close();
    }

    if (m_LockFd >= 0)
    {
        flock(m_LockFd, LOCK_UN);
        ::close(m_LockFd);
        m_LockFd = -1;
    }

    internalFilePath.clear();
    m_Size = 0;
    m_Offset = 0;
}

std::vector<char> CFile::read(size_t length)
{
    std::vector<char> buffer;
    if (!m_FileStream.is_open())
    {
        return buffer; // Return empty buffer if file is not open
    }

    if (length == 0)
    {
        // Guard if the caller wants to read the entire file and there is not
        // enough memory to hold it.
        struct sysinfo si;
        if (sysinfo(&si) == 0)
        {
            unsigned long free_ram = si.freeram * si.mem_unit;
            if (free_ram < m_Size)
            {
                return buffer; // Not enough memory to read the entire file
            }
        }
        length = m_Size;
    }

    if (length == 0 || length > m_Size)
    {
        length = m_Size; // Clamp to available file size
    }

    buffer.resize(length);

    m_FileStream.seekg(m_Offset, std::ios::beg);
    m_FileStream.read(buffer.data(), static_cast<std::streamsize>(length));
    std::streamsize bytesRead = m_FileStream.gcount();
    if (bytesRead > 0)
    {
        m_Offset += static_cast<size_t>(bytesRead);
    }

    buffer.resize(static_cast<size_t>(bytesRead));
    return buffer;
}

int CFile::GetSha256Hash(std::vector<char>& hashOutput)
{
    if (!m_FileStream.is_open())
    {
        return -1; // File not open
    }

    // Save current position so we can restore it afterwards.
    std::streampos originalPos = m_FileStream.tellg();

    m_FileStream.seekg(0, std::ios::end);
    std::streampos fileSizePos = m_FileStream.tellg();
    m_FileStream.seekg(0, std::ios::beg);

    if (fileSizePos < 0)
    {
        return -1; // Error getting file size
    }

    auto fileSize = static_cast<size_t>(fileSizePos);

    hashOutput.resize(SHA256_DIGEST_LENGTH);

    if (fileSize == 0)
    {
        // Hash of empty input.
        SHA256(nullptr, 0, reinterpret_cast<unsigned char*>(hashOutput.data()));
    }
    else
    {
        std::vector<char> buffer(fileSize);
        m_FileStream.read(buffer.data(), static_cast<std::streamsize>(fileSize));
        if (static_cast<size_t>(m_FileStream.gcount()) != fileSize)
        {
            hashOutput.clear();
            m_FileStream.seekg(originalPos, std::ios::beg);
            return -1; // Failed to read entire file
        }

        SHA256(reinterpret_cast<const unsigned char*>(buffer.data()), buffer.size(),
               reinterpret_cast<unsigned char*>(hashOutput.data()));
    }

    m_FileStream.seekg(originalPos, std::ios::beg);
    return 0; // Success
}

size_t CFile::write(const uint8_t* data, size_t length)
{
    if (!m_FileStream.is_open() || length == 0)
    {
        return 0; // File not open or nothing to write
    }

    m_FileStream.seekp(m_Offset, std::ios::beg);
    m_FileStream.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(length));
    m_FileStream.flush();

    if (!m_FileStream.good())
    {
        return 0; // Write failed
    }

    m_Offset += length;
    if (m_Offset > m_Size)
    {
        m_Size = m_Offset;
    }

    return length; // Return number of bytes written
}

int CFile::remove()
{
    if (internalFilePath.empty())
    {
        return -1; // No file path available
    }
    std::string filePath = internalFilePath;
    close();
    return std::remove(filePath.c_str());
}

int CFile::RemoveAll(char* path, char* wildCard)
{
    if ((path == nullptr) || (wildCard == nullptr))
    {
        return -1; // Invalid arguments
    }

    std::filesystem::path dirPath(path);
    if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath))
    {
        return -1; // Directory does not exist
    }

    std::string pattern(wildCard);
    std::string regexPattern = std::regex_replace(pattern, std::regex(R"(\.)"), R"(\.)");
    regexPattern = std::regex_replace(regexPattern, std::regex(R"(\*)"), R"(.*)");
    regexPattern = std::regex_replace(regexPattern, std::regex(R"(\?)"), R"(.)");
    std::regex fileRegex(regexPattern);
    
    // Iterate through the directory and remove matching files
    for (const auto& entry : std::filesystem::directory_iterator(dirPath))
    {
        if (std::filesystem::is_regular_file(entry.status()))
        {
            std::string fileName = entry.path().filename().string();
            if (std::regex_match(fileName, fileRegex))
            {
                std::error_code ec;
                if (ec)
                std::filesystem::remove(entry.path(), ec);
                {
                    return -1; // Failed to remove a file
                }
            }
        }
    }
    return 0;
}

int CFile::IsFileAvailable(const char* filePath)
{
    if (filePath == nullptr)
    {
        return -1; // Invalid argument
    }

    std::filesystem::path filePathObj(filePath);
    if (!std::filesystem::exists(filePathObj))
    {
        return 0; // File does not exist
    }

    int lockFd = ::open(filePath, O_RDWR);
    if (lockFd < 0)
    {
        return -1; // Failed to open the file
    }

    if (flock(lockFd, LOCK_EX | LOCK_NB) < 0)
    {
        // Someone else already has this file open.
        ::close(lockFd);
        return -1;
    }
    ::close(lockFd);
    return 0; // File is available
}