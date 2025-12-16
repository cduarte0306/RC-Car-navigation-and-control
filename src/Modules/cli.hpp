#ifndef CLI_HPP
#define CLI_HPP

#include <Modules/RcBase.hpp>
#include "embedded_cli.h"


namespace Modules {
class AppCLI : public Modules::Base {
public:
    AppCLI(int moduleID_, std::string name);
    ~AppCLI();
    // Helper to forward a command buffer to the motor adapter
    int sendMotorCommand(char* pbuf, size_t len);
    
    virtual int init(void) override {
        return 0;
    }
    virtual int stop(void) override;

protected:
    virtual void mainProc() override;
private:
    const char* tty = "/dev/ttyTHS1";

    EmbeddedCli *CLI = nullptr;
    int fd = -1;

    int openInterface();
    int writeIface(const char* format, ...);
};
}

#endif