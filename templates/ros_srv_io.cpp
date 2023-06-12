#include <ros_msg_io.h>
#include <ros_srv_io.h>
#include <simLib/simLib.h>

#py from parse_messages_and_services import get_srvs_info
#py srvs = get_srvs_info(pycpp.params['services_file'])
#py for srv, info in srvs.items():
bool ros_srv_callback__`info.typespec.normalized()`(`info.typespec.ctype()`::Request& req, `info.typespec.ctype()`::Response& res, ServiceServerProxy *proxy)
{
    bool ret = false;
    int stack = -1;

    try
    {
        stack = sim::createStack();
        write__`info.typespec.normalized()`Request(req, stack, &(proxy->wr_opt));
        sim::callScriptFunctionEx(proxy->serviceCallback.scriptId, proxy->serviceCallback.name.c_str(), stack);
        read__`info.typespec.normalized()`Response(stack, &res, &(proxy->rd_opt));
        sim::releaseStack(stack);
        stack = -1;
        return true;
    }
    catch(std::exception &ex)
    {
        if(stack != -1)
            simReleaseStack(stack); // don't throw
        std::string msg = "ros_srv_callback__`info.typespec.normalized()`: ";
        msg += ex.what();
        sim::setLastError(msg);
        return false;
    }
}

#py endfor
