# C++ plugin framework for V-REP

Compile with your C++ project.

Example plugin (uses also [v_repStubsGen](https://github.com/CoppeliaRobotics/v_repStubsGen)):

```
#include "simExtPluginSkeletonNG.h"
#include "v_repPlusPlus/Plugin.h"
#include "stubs.h"

void test(SScriptCallBack *p, const char *cmd, test_in *in, test_out *out)
{
    // ...
}

class Plugin : public vrep::Plugin
{
public:
    void onStart()
    {
        if(!registerScriptStuff())
            throw std::runtime_error("script stuff initialization failed");
    }
};

VREP_PLUGIN("PluginSkeletonNG", 1, Plugin)
```

See [simExtPluginSkeletonNG](https://github.com/CoppeliaRobotics/simExtPluginSkeletonNG) for a complete example.
