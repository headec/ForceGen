#include <unistd.h>
#include "dhdc.h"
#include "drdc.h"

int main()
{
    drdOpen();
    drdStart();
    sleep(86400);
    return 0;
}
