#pragma once
#include <QtCore/qglobal.h>

#if defined(AUTOCONTROLDLL_LIBRARY)
#  define AUTOCONTROLDLL_API Q_DECL_EXPORT
#else
#  define AUTOCONTROLDLL_API Q_DECL_IMPORT
#endif