#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include <vector>

#include "comms.h"
#include "pruThread.h"

vector<PRUThread*> configure_threads(const Comms* comms);

#endif