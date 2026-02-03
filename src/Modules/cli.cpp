#include <iostream>
#include <sstream>

#include <thread>

#include "cli.hpp"
// #include "../peripheral_driver.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdarg>
#include <sys/reboot.h>
#include <vector>

#include "version.h"
#include "utils/logger.hpp"
#include "../types.h"

#include "RcMotorController.hpp"


/* Definations for CLI configurations */
#define CLI_BUFFER_SIZE     (256u)
#define RX_BUFFER_SIZE      (32u)                   // To store chars untill they're proccessed
#define CMD_BUFFER_SIZE     (32u)                   // To store current input that is not yet sent as command
#define HISTORY_BUFFER_SIZE (4 * CMD_BUFFER_SIZE)   // To store previous commands

#define CLI_PORT            (65000)

namespace Modules
{
AppCLI::AppCLI(int moduleID_, std::string name) : Base(moduleID_, name) {
    if (this->openInterface() < 0) {
        throw("Failed to open TTY interface");
    }

    this->writeIface("\033[2J\033[H");
    this->writeIface("\r\n*************************RC Car CLI Interface*************************\r\n");
    this->writeIface("Software version: %u.%u.%u\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD);
    this->writeIface("Please enter \"help\" to list available commands\r\n");

    const CliCommandBinding cmdBindings[] = {
        (CliCommandBinding){
            "read-psoc",
            
            "Reads PSoC data\r\n"
                "\t\tread-psoc\r\n",
            
            false, this,
            
            [](EmbeddedCli *cli, char *args, void *context) {
                typedef struct {
                    val_type_t version_major;   // Major version
                    val_type_t version_minor;   // Minor version
                    val_type_t version_build;   // Build version
                    val_type_t speed;          // Speed in Hz
                    val_type_t frontDistance;  // Distance in mm
                    val_type_t leftDistance;   // Distance in mm
                    val_type_t rightDistance;  // Distance in mm
                } psocDataStruct;

                (void)args;
                (void)cli;
                char buff[1024];
                Modules::MotorController::MotorCommand_t* cmd = reinterpret_cast<Modules::MotorController::MotorCommand_t*>(buff);
                cmd->command = Modules::MotorController::MotorCmdReadData;
                
                AppCLI* _cli = static_cast<AppCLI*>(context);
                int ret = _cli->motorAdapter->moduleCommand(buff, sizeof(buff));
                if (ret < 0) {
                    _cli->writeIface("Failed to read register: %d\r\n", cmd->data_1.u8);
                }

                psocDataStruct* psocData = reinterpret_cast<psocDataStruct*>(buff + sizeof(Modules::MotorController::MotorCommand_t));
                std::stringstream out;
                out << "Version: "
                << static_cast<int>(psocData->version_major.u8) << "."
                << static_cast<int>(psocData->version_minor.u8) << "."
                << static_cast<int>(psocData->version_build.u8) << "\r\n"
                << "Speed: " << psocData->speed.f32 << "\r\n"
                << "Front distance: " << psocData->frontDistance.f32 << "\r\n"
                << "Left distance: " << psocData->leftDistance.f32 << "\r\n"
                << psocData->rightDistance.f32;
                _cli->writeIface("%s\r\n", out.str().c_str());
            }
        },
        (CliCommandBinding){
            "write-spi",
            
            "Writes SPI data\r\n"
                "\twrite-spi \"<data>\"\r\n",
            
            false, this,
            
            [](EmbeddedCli *cli, char *args, void *context) {
                (void)args;
                (void)cli;
                AppCLI* _cli = static_cast<AppCLI*>(context);
                std::vector<uint8_t> array;
                
                int index = 1;
                const char *arg1 = nullptr;
                while(embeddedCliGetToken(args, index)) {
                    arg1 = embeddedCliGetToken(args, index++);
                    array.push_back(std::stoi(std::string(arg1)));
                }
                char buff[1024];
                Modules::MotorController::MotorCommand_t* cmd = reinterpret_cast<Modules::MotorController::MotorCommand_t*>(buff);
                cmd->command = Modules::MotorController::MotorCmdSpiWrite;
                if (!arg1) {
                    _cli->writeIface("ERROR: Failed to provide argument\r\n");
                    return;
                }

                int ret = _cli->motorAdapter->moduleCommand(buff, sizeof(buff));
            }
        },
        (CliCommandBinding){
            "read-spi-reg",
            
            "Reads SPI register\r\n"
                "\tread-spi-reg \"<reg>\"\r\n",
            true, this,
            
            [](EmbeddedCli *cli, char *args, void *context) {
                (void)cli;
                const char *reg_ = embeddedCliGetToken(args, 1);
                AppCLI* _cli = static_cast<AppCLI*>(context);

                int reg = std::stoi(std::string(reg_));
                if (reg_ == nullptr) {
                    _cli->writeIface("ERROR: Failed to provide argument\r\n");
                    return;
                }
                
                char buff[1024];
                Modules::MotorController::MotorCommand_t* cmd = reinterpret_cast<Modules::MotorController::MotorCommand_t*>(buff);
                cmd->command = Modules::MotorController::MotorCmdSpiRead;
                int ret = _cli->motorAdapter->moduleCommand(buff, sizeof(buff));
                if (ret < 0) {
                    _cli->writeIface("Failed to read register: %d\r\n", cmd->data_1.u8);
                }
                _cli->writeIface("Register %d: %u\r\n", reg, cmd->data_1.u8);
            }
        },
        (CliCommandBinding){
            "read-stats",
            
            "Reads stats from specied module\r\n"
                "\tCommunications module ID: 0\r\n"
                "\tread-stats \"<module ID>\"\r\n",
            true, this,
            
            [](EmbeddedCli *cli, char *args, void *context) {
                (void)cli;
                AppCLI* _cli = static_cast<AppCLI*>(context);
                const char *moduleIdx_ = embeddedCliGetToken(args, 1);
                if (moduleIdx_ == nullptr) {
                    _cli->writeIface("ERROR: Failed to provide argument\r\n");
                    return;
                }
                
                int moduleIdx = std::stoi(std::string(moduleIdx_));
                switch (moduleIdx)
                {
                case 0: {
                    std::string stats = _cli->CommsAdapter->readStats();
                    _cli->writeIface("%s\n", stats.c_str());
                    break;   
                }

                default:
                    _cli->writeIface("Unknown module ID provided\r\n");
                    break;
                }
            }
        },
        (CliCommandBinding){
            "reboot-cpu",
            
            "Reboots CPU after shutting off all RC car components\r\n"
                "\t\treboot-cpu\r\n",
            
            false, this,
            
            [](EmbeddedCli *cli, char *args, void *context) {
                (void)args;
                (void)cli;
                AppCLI* _cli = static_cast<AppCLI*>(context);
                _cli->writeIface("Rebooting CPU. This may take up to 2 minutes...\r\n");
                reboot(RB_AUTOBOOT);
            }
        }
    };

    const uint8_t num_cmds = sizeof( cmdBindings )/sizeof( cmdBindings[0] );
    bool ret;
    uint16_t req_size;

    EmbeddedCliConfig *config = embeddedCliDefaultConfig();
    if ( config == NULL ) {
        throw("config is nullptr");
    }

    config->invitation         = "rc-car> ";
    config->cliBuffer           = NULL;
    config->cliBufferSize       = CLI_BUFFER_SIZE * sizeof( CLI_UINT );
    config->rxBufferSize        = RX_BUFFER_SIZE;
    config->cmdBufferSize       = CMD_BUFFER_SIZE;
    config->historyBufferSize   = HISTORY_BUFFER_SIZE;
    config->maxBindingCount     = num_cmds + 1;
    config->enableAutoComplete  = true;

    this->CLI = embeddedCliNew( config );
    if ( CLI == NULL )
    {
        throw(0);
    }

    this->CLI->appContext = this;

    for ( uint8_t i = 0; i<num_cmds; i++ )
    {
        ret = embeddedCliAddBinding( CLI, cmdBindings[i] );
        if (ret == false)
        {
            throw(0);
        }

        if (CLI == NULL)
        {
            throw(0);
        }
    }

    // Override the writeChar function to send output over UDP
    CLI->writeChar = [](EmbeddedCli *cli, char c ) {
        AppCLI* _cli = static_cast<AppCLI*>(cli->appContext);
        int ret = write(_cli->fd, &c, 1);
        Logger* logger = Logger::getLoggerInst();
        if (ret < 0) {
            logger->log(Logger::LOG_LVL_ERROR, "%s, %s, Error writing to TTY\r\n", __func__, __LINE__);
        }
    };

    Logger* logger = Logger::getLoggerInst();
    logger->log(Logger::LOG_LVL_INFO, "CLI Initialized...\r\n");
}


AppCLI::~AppCLI() {
}

int AppCLI::stop(void) {
    // Stop main loop and close TTY
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
    return 0;
}


void AppCLI::mainProc() {
    uint8_t buffer[128];
        
    // Simulate the enter key press to show the invitation prompt
    embeddedCliReceiveChar(this->CLI, '\n');
    embeddedCliProcess(this->CLI);

    while (true) {
        int ret = read(this->fd, buffer, sizeof(buffer));
        if (ret < 0) {
            std::cerr << "Error reading from TTY" << std::endl;
            continue;
        }
        for (int i = 0; i < ret; ++i) {
            embeddedCliReceiveChar(this->CLI, buffer[i]);
        }
        embeddedCliProcess(this->CLI);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


/**
 * @brief Opens the TTY interface
 * 
 * @return int File descriptor of opened TTY, or -1 on error
 */
int AppCLI::openInterface() {
    this->fd = open(this->tty, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open " << this->tty << std::endl;
        return -1;
    }

    fcntl(fd, F_SETFL, 0); // Blocking read

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Clear current char size mask
    options.c_cflag |= CS8;              // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    options.c_oflag &= ~OPOST;                          // Raw output

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}


/**
 * @brief Writes formatted data to the TTY interface
 * 
 * @param format String format
 * @param ... 
 * @return int Number of bytes written, or -1 on error
 */
int AppCLI::writeIface(const char* format, ...) {
    if (this->fd < 0) {
        std::cerr << "TTY interface not opened" << std::endl;
        return -1;
    }

    char buffer[1024];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len < 0) {
        std::cerr << "Error formatting string" << std::endl;
        return -1;
    }
    int ret = write(this->fd, buffer, len);
    if (ret < 0) {
        std::cerr << "Error writing to TTY" << std::endl;
        return -1;
    }
    return ret;
}
}