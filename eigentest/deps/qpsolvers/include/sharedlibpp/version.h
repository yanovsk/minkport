/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef SHAREDLIBPP_VERSION_H
#define SHAREDLIBPP_VERSION_H

#include <sharedlibpp/api.h>
#include <string>

#define SHLIBPP_VERSION_MAJOR 0
#define SHLIBPP_VERSION_MINOR 0
#define SHLIBPP_VERSION_PATCH 3
#define SHLIBPP_VERSION "0.0.3"

namespace sharedlibpp {

SHLIBPP_API int getVersionMajor();
SHLIBPP_API int getVersionMinor();
SHLIBPP_API int getVersionPatch();
SHLIBPP_API std::string getVersion();

} // namespace sharedlibpp

#endif // SHAREDLIBPP_VERSION_H
