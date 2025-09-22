#include <iostream>
#include <sstream>

#include "cli.hpp"
#include "../peripheral_driver.hpp"

AppCLI::AppCLI(RcCar& mainObj) : mainObj(mainObj) {
    
    // Initialize the embedded CLI
    AppCLI cli(this->mainObj);

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
                
                std::cout << out.str();
            }
        }
    };
}


AppCLI::~AppCLI() {

}