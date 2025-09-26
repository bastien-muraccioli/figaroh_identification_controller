#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define FigarohIdentificationController_DLLIMPORT __declspec(dllimport)
#  define FigarohIdentificationController_DLLEXPORT __declspec(dllexport)
#  define FigarohIdentificationController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define FigarohIdentificationController_DLLIMPORT __attribute__((visibility("default")))
#    define FigarohIdentificationController_DLLEXPORT __attribute__((visibility("default")))
#    define FigarohIdentificationController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define FigarohIdentificationController_DLLIMPORT
#    define FigarohIdentificationController_DLLEXPORT
#    define FigarohIdentificationController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef FigarohIdentificationController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define FigarohIdentificationController_DLLAPI
#  define FigarohIdentificationController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef FigarohIdentificationController_EXPORTS
#    define FigarohIdentificationController_DLLAPI FigarohIdentificationController_DLLEXPORT
#  else
#    define FigarohIdentificationController_DLLAPI FigarohIdentificationController_DLLIMPORT
#  endif // FigarohIdentificationController_EXPORTS
#  define FigarohIdentificationController_LOCAL FigarohIdentificationController_DLLLOCAL
#endif // FigarohIdentificationController_STATIC
