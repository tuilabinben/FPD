// The firmware does #include <Ethernet.h> when PLC_LINK_MODE is
// PLC_LINK_ETHERNET. Everything that header would provide is already
// declared in ClearCore.h for the desktop build, so this exists only so
// the include resolves. Keeping it empty rather than duplicating the
// declarations means there is one definition of the shim, not two that
// can drift.
#pragma once
