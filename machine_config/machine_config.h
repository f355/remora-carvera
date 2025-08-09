#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include <vector>

#include "PRUThread.h"
#include "comms.h"

vector<PRUThread*> configure_threads(const Comms* comms);

#endif