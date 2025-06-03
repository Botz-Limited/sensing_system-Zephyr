/**
 * @file safe_buffer_error.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */

/**
 *****************************************************************************************************************************************************
 *  \section HEADER_GUARD
 *****************************************************************************************************************************************************
 */
#ifndef ERROR_H_
#define ERROR_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include "chp_lib.hpp"


/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */
/**
 *  A block structure used for ensuring a group of functions all complete successfully.  If they all succeed x will be set to RESULT_OK, otherwise it
 *  will be set to RESULT_ERROR.  The value of x can then be tested by the CATCH macro.
 */
#define TRY(x)              for ((x) = RESULT_ERROR; (x) == RESULT_ERROR; (x) = RESULT_OK)

/**
 *  A control block that tests the value of x from the TRY statement.  If it is set to RESULT_ERROR then the control block will be executed, which
 *  may contain corrective actions, logging, or error reporting of some kind.
 */
#define CATCH(x)            if ((x) == RESULT_ERROR)

/** A finally block executes regardless of whether the TRY completed successfully or not. */
#define FINALLY(x)          ;

/** Calls the given function and checks the return value.  If the function fails execution will break out of the TRY block. */
#define ATTEMPT(f)          if ((f) == RESULT_ERROR) { break; } \
        else { }

/** Calls the given function n times, using the given loop counter i.  If a function fails execution will break out of the TRY block.   */
#define ITERATE(f, i, n)    for ((i) = 0U; (i) < (n); (i)++) {ATTEMPT(f);} \
        ENSURE((i) == (n))

/** Checks if the given condition is true.  If the condition fails execution will break out of the TRY block. */
#define ENSURE(c)           if (!(c)) { break; } \
        else { }

/** Checks if the given condition is true.  If the condition fails execution will break out of the TRY block and execute func */
#define ENSURE_OR(c, func)  if (!(c)) { do { (func); } while (0); break; } \
        else { }


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL TYPEDEFS
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL VARIABLES
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */


#ifdef __cplusplus
}
#endif


#endif  /* ERROR_H_ */


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
