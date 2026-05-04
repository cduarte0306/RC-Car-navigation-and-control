#include "CFile.hpp"
#include <sys/sysinfo.h>
#include <filesystem>
#include <openssl/sha.h>


CFile::CFile(const char* filePath, const char* mode) {
    m_File = fopen(filePath, mode);
}

CFile::~CFile() {
    if (m_File) {
        fclose(m_File);
    }
}

int CFile::open(const char* filePath, const char* mode) {
    if (m_File) {
        fclose(m_File);
    }


    m_File = fopen(filePath, mode);
    return m_File ? 0 : -1;
}

void CFile::close() {
    if (m_File) {
        fclose(m_File);
        m_File = nullptr;
        m_Size = 0;
    }
}

std::vector<char> CFile::read(size_t length) {
    std::vector<char> buffer;
    if (!m_File) {
        return buffer; // Return empty buffer if file is not open
    }

    // Guard if the caller wants to read the entire file and there is not enough memory to hold it
    if (length == 0) {
        struct sysinfo si;
        if (sysinfo(&si) == 0) {
            // Multiply by mem_unit to get actual bytes (unit might be 1, 1024, etc.)
            unsigned long total_ram = si.totalram * si.mem_unit;
            unsigned long free_ram = si.freeram * si.mem_unit;
            
            if (free_ram < m_Size) {
                return buffer; // Not enough memory to read the entire file, return empty buffer
            }
        }

        fseek(m_File, 0, SEEK_END);
        long fileSize = ftell(m_File);
        fseek(m_File, 0, SEEK_SET);
        if (fileSize < 0) {
            return buffer; // Return empty buffer on error
        }

        
        length = static_cast<size_t>(fileSize);
    }

    // Get file size
    fseek(m_File, 0, SEEK_END);
    m_Size = ftell(m_File);
    fseek(m_File, 0, SEEK_SET);

    if (length == 0 || length > m_Size) {
        length = m_Size; // Read entire file if length is 0 or exceeds file size
    }

    buffer.resize(length);
    fread(buffer.data(), sizeof(char), length, m_File);
    return buffer;
}

int CFile::GetSha256Hash(std::vector<uint8_t>& hashOutput) {
    if (!m_File) {
        return -1; // File not open
    }

    // Get file size
    fseek(m_File, 0, SEEK_END);
    long fileSize = ftell(m_File);
    fseek(m_File, 0, SEEK_SET);

    if (fileSize < 0) {
        return -1; // Error getting file size
    }

    std::vector<char> buffer(fileSize);
    fread(buffer.data(), sizeof(char), fileSize, m_File);

    hashOutput.resize(SHA256_DIGEST_LENGTH);
    SHA256(reinterpret_cast<const unsigned char*>(buffer.data()), buffer.size(), hashOutput.data());
    
    return 0; // Success
}

size_t CFile::write(const uint8_t* data, size_t length, size_t offset) {
    if (!m_File) {
        return 0; // File not open
    }

    fseek(m_File, offset, SEEK_SET);
    size_t bytesWritten = fwrite(data, sizeof(uint8_t), length, m_File);
    m_Size = std::max(m_Size, offset + bytesWritten); // Update file size if we wrote past the previous end
    return bytesWritten;
}