#ifndef _APP_VERSION_H_
#define _APP_VERSION_H_

/* The template values come from cmake/version.cmake
 * BUILD_VERSION related template values will be 'git describe',
 * alternatively user defined BUILD_VERSION.
 */

/* #undef ZEPHYR_VERSION_CODE */
/* #undef ZEPHYR_VERSION */

#define APPVERSION                   0x1000100
#define APP_VERSION_NUMBER           0x10001
#define APP_VERSION_MAJOR            1
#define APP_VERSION_MINOR            0
#define APP_PATCHLEVEL               1
#define APP_TWEAK                    0
#define APP_VERSION_STRING           "1.0.1"
#define APP_VERSION_EXTENDED_STRING  "1.0.1+0"
#define APP_VERSION_TWEAK_STRING     "1.0.1+0"

#define APP_BUILD_VERSION 5f62d5b2aaac


#endif /* _APP_VERSION_H_ */
