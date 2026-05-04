#ifndef CLI_HPP
#define CLI_HPP

#include <Modules/RcBase.hpp>
#include "embedded_cli.h"


namespace Modules {
class AppCLI : public Modules::Base {
public:
    AppCLI(ModuleDefs::DeviceType moduleID_, std::string name);
    ~AppCLI();
    // Helper to forward a command buffer to the motor adapter
    int sendMotorCommand(char* pbuf, size_t len);
    
    virtual int init(void) override;
    virtual int stop(void) override;

protected:
    virtual void mainProc() override;

    std::unique_ptr<Adapter::CommsAdapter::NetworkAdapter> m_CliAdapter{nullptr};

    std::mutex mutex;
    std::vector<char> cmdBuffer;
private:
    const char* tty = "/dev/ttyTHS1";

    EmbeddedCli *CLI = nullptr;
    int fd = -1;

    
    int writeIface(const char* format, ...);
};
}

#endif