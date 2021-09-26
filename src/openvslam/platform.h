#ifndef  OPENVSLAM_PLATFORM_H
#define  OPENVSLAM_PLATFORM_H

#ifdef MSVC
#	ifdef OPENVSLAM_EXPORT
#       define DECLSPEC __declspec(dllexport)
#   else
#       define DECLSPEC __declspec(dllimport)
#   endif
#else
#   define DECLSPEC 
#endif

#endif //  OPENVSLAM_PLATFORM_H
