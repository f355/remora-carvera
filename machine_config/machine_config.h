#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include <vector>

#include "MachineThread.h"
#include "comms.h"

vector<MachineThread*> configure_threads(Comms* comms);

#endif