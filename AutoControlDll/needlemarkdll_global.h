#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(NEEDLEMARKDLL_LIB)
#  define NEEDLEMARKDLL_EXPORT Q_DECL_EXPORT
# else
#  define NEEDLEMARKDLL_EXPORT Q_DECL_IMPORT
# endif
#else
# define NEEDLEMARKDLL_EXPORT
#endif
