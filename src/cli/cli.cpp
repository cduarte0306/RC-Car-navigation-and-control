#include <iostream>
#include <sstream>

#include <thread>

#include "cli.hpp"
#include "../peripheral_driver.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


/* Definations for CLI configurations */
#define CLI_BUFFER_SIZE     (256u)
#define RX_BUFFER_SIZE      (32u)                   // To store chars untill they're proccessed
#define CMD_BUFFER_SIZE     (32u)                   // To store current input that is not yet sent as command
#define HISTORY_BUFFER_SIZE (4 * CMD_BUFFER_SIZE)   // To store previous commands

#define CLI_PORT            (65000)


AppCLI::AppCLI(RcCar& mainObj) : mainObj(mainObj) {
    if (this->openInterface() < 0) {
        throw("Failed to open TTY interface");
    }

    const CliCommandBinding cmdBindings[] = {
        (CliCommandBinding){
            "read-psoc",
            
            "Reads PSoC data\r\n"
                "\t\tread-psoc\r\n",
            
            false, this,
            
            [](EmbeddedCli *cli, char *args, void *context) {
                AppCLI* _cli = static_cast<AppCLI*>(context);
                RcCar& rcCar = _cli->mainObj;
                PeripheralCtrl* psoc = rcCar.getModule<PeripheralCtrl>();
                PeripheralCtrl::psocDataStruct psocData;
                int ret = psoc->readData(psocData);
                
                std::stringstream out;
                out << "Version: " << psocData.version_major.u8 << "." << psocData.version_minor.u8 << "." << psocData.version_build.u8 << "\r\n"
                "Speed: " << psocData.speed.f32 << "\r\n" << "Front distance: " << psocData.frontDistance.f32 << "\r\n" << 
                "Left distance: " << psocData.leftDistance.f32 << "\r\n" << psocData.rightDistance.f32 << "\r\n";
                std::cout << out.str().c_str() << std::endl;
                std::string data = out.str();
                _cli->writeIface(data);
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
        if (ret < 0) {
            std::cerr << "Error writing to TTY" << std::endl;
        }
    };

    std::thread cliThread([this]() {
        uint8_t buffer[128];
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
    });
    cliThread.detach();
}


AppCLI::~AppCLI() {

}


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


int AppCLI::writeIface(std::string& data) {
    if (this->fd < 0) {
        std::cerr << "TTY interface not opened" << std::endl;
        return -1;
    }

    int len = data.length();
    int ret = write(this->fd, data.c_str(), len);
    if (ret < 0) {
        std::cerr << "Error writing to TTY" << std::endl;
        return -1;
    }
    return ret;
}