#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include <vector>

#include "Comms.h"
#include "PRUThread.h"

vector<PRUThread*> configure_threads(Comms* comms);

#endif