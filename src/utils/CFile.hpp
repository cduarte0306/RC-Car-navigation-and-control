#pragma once

#include <string>
#include <vector>
#include <cstdint>

class CFile {
public:
    CFile() = default;
    CFile(const char* filePath, const char* mode);
    ~CFile();

    /**
     * @brief Open a file with the specified path and mode
     * 
     * @param filePath Path to the file to open
     * @param mode File open mode (e.g., "r", "w", "rb", "wb")
     * @return int 0 on success, -1 on failure
     */
    int open(const char* filePath, const char* mode);

    /**
     * @brief Open a file with the specified path and mode (overload for std::string)
     * 
     * @param filePath Path to the file to open
     * @param mode File open mode (e.g., "r", "w", "rb", "wb")
     * @return int 0 on success, -1 on failure
     */
    int open(std::string filePath, std::string mode) {
        return open(filePath.c_str(), mode.c_str());
    }

    /**
     * @brief Close the file if it is open
     */
    void close();

    /**
     * @brief Write data to the file (overload for std::vector<char>)
     * 
     * @param buffer Buffer containing the data to write
     * @return size_t Number of bytes actually written
     */
    size_t write(const std::vector<char>& buffer, size_t offset = 0) {
        return write(reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size(), offset);
    }

    /**
     * @brief Read data from the file
     * 
     * @param length Number of bytes to read; if 0, read the entire file
     * @return std::vector<char> Buffer containing the read data
     */
    std::vector<char> read(size_t length=0);

    /**
     * @brief Get the size of the file in bytes
     * 
     * @return size_t Size of the file
     */
    size_t size() const {
        return m_Size;
    }

    /**
     * @brief Check if the file is open
     * 
     * @return true if the file is open, false otherwise
     */
    bool isOpen() const {
        return m_File != nullptr;
    }

    /**
     * @brief Get the Sha 256 hash of the file contents
     * 
     * @param hashOutput 
     * @return int 
     */
    int GetSha256Hash(std::vector<uint8_t>& hashOutput);

private:

    /**
     * @brief Write data to the file
     * 
     * @param data Pointer to the data to write
     * @param length Number of bytes to write
     * @return size_t Number of bytes actually written
     */
    size_t write(const uint8_t* data, size_t length, size_t offset = 0);

    
    FILE* m_File = nullptr;

    size_t m_Size = 0;  // Size of the file in bytes
};

#pragma endregion