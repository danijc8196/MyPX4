#include <px4_log.h>

__EXPORT int new_app_main(int argc, char *argv[]);

int new_app_main(int argc, char *argv[])
{
    PX4_INFO("Hello Sky!");
    return OK;
}
