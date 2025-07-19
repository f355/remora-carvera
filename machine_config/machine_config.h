#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include <vector>

#include "cncThread.h"
#include "comms.h"

vector<CNCThread*> configure_threads(Comms* comms);

#endif