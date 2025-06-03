/**
 * @file crc32.cpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */



/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include <string.h>
#include <safe_buffer_error.hpp>

#include <safe_buffer.hpp>


/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section STATIC FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section STATIC VARIABLES
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL VARIABLES
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section SOURCE CODE
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  @brief  Checks members of target buffer to ensure it's safe for use
 *  @param  [in]  target - The safe buffer to be validated
 *  @return RESULT_OK if successful, RESULT_ERROR otherwise
 *****************************************************************************************************************************************************
 */
result_t safe_buffer_Valid(const safe_buffer_t* const target)
{
    result_t valid;

    TRY(valid)
    {
        ENSURE(target != NULL);             /**< Do not dereference pointer unless checked for NULL first   */
        ENSURE(target->buffer != NULL);     /**< buffer member must be a valid pointer                      */
        ENSURE(target->length != NULL);     /**< length member must be a valid pointer                      */
        ENSURE(target->max_length > 0U);    /**< max_length must be greater than zero to make sense         */
    }

    return valid;
}


/**
 *****************************************************************************************************************************************************
 *  @brief  A safe memcpy, effectively.  Prototype is designed to mimic original memcpy call.
 *  @param  [out]   dest    - Destination safe buffer
 *  @param  [in]    src     - Source safe buffer
 *  @param  [in]    len     - 1..MAX_UINT = Number of bytes to copy.  0 = source's length, not caring if all of it fits in destination
 *  @return RESULT_OK if all bytes requested were copied, RESULT_ERROR otherwise
 *****************************************************************************************************************************************************
 */
result_t safe_buffer_Copy(safe_buffer_t* const dest, const safe_buffer_t* const src, uint32_t len)
{
    result_t copied;
    uint32_t requested_length;
    uint32_t copy_length;

    /* TODO: This function can handle the following scenarios -
        1. User wants x amount of bytes copied safely and needs to know if it worked
        2. User wants all of source copied up to the point destination becomes full

       But what if the user doesn't want the memcpy to occur if there's no room?  We cannot use len to imply this.
       Do we need another parameter (boolean_t must_fit)?
     */

    TRY(copied)
    {
        /* Check for valid safe buffers first */
        ENSURE(safe_buffer_Valid(dest));
        ENSURE(safe_buffer_Valid(src));

        /* If len == 0, caller asked us to copy all of src's buffer even if it gets cut short because of dest's buffer size */
        requested_length = (len == 0U) ? *src->length : len;

        /* Determine how many bytes can be *safely* copied from src -> dest */
        copy_length = (requested_length <= dest->max_length) ? requested_length : dest->max_length;

        /*** The dangerous function ***/
        memcpy(dest->buffer, src->buffer, copy_length);

        /* Save copied amount into dest's length field */
        *dest->length = copy_length;

        /* This may seem odd here but although we haven't overflowed or caused any other error,
           we need to indicate whether or not the full copy was successful, unless the caller didn't care (len == 0). */
        if (len != 0U)
        {
            ENSURE(copy_length == len);
        }
    }

    return copied;
}


/**
 *****************************************************************************************************************************************************
 *  @brief  A safe strncpy, effectively.  Prototype is designed to mimic original strncpy call.
 *  @param  [out]   dest    - Destination safe buffer
 *  @param  [in]    src     - Source string
 *  @param  [in]    len     - 1..MAX_UINT = Number of bytes to copy up to.  All extra bytes after src length will be NUL
 *  @return RESULT_OK if all bytes requested were copied, RESULT_ERROR otherwise
 *****************************************************************************************************************************************************
 */
result_t safe_buffer_StringNullCopy(char_t* const dest, const char_t* const src, uint32_t len)
{
    result_t copied;

    TRY(copied)
    {
        /* Run it through the library strncpy - this will do array bound limiting for us */
        strncpy(dest, src, len);

        /* What strncpy *doesn't* do, however, is null terminate our dest.
           Now, we don't know how long src was (and using strlen(src) is part of what the problem is if not null terminated)
           so because strncpy nulls up to len, we only need to null the last index in case strncpy didn't get the chance to do so.

           The outcome?
           strlen(dest) will either be strlen(src) if src terminated correctly or len if src didn't terminate in time.
           Either way we performed the operation safely and subsequent buffer use shouldn't fail us. */
        dest[len - 1U] = '\0';
    }

    return copied;
}


/**
 *****************************************************************************************************************************************************
 *  @brief  Append the content of one safe buffer to another
 *  @param  [out]   dest    - Destination safe buffer
 *  @param  [in]    src     - Source safe buffer
 *  @param  [in]    len     - 1..MAX_UINT = Number of bytes to append.  0 = source's length, not caring if all of it fits in destination
 *  @return RESULT_OK if all bytes requested were copied, RESULT_ERROR otherwise
 *****************************************************************************************************************************************************
 */
result_t safe_buffer_Append(safe_buffer_t* const dest, const safe_buffer_t* const src, uint32_t len)
{
    result_t appended;
    uint32_t requested_length;
    uint32_t final_length;
    TRY(appended)
    {
        /* Check for valid safe buffers first */
        ENSURE(safe_buffer_Valid(dest));
        ENSURE(safe_buffer_Valid(src));

        /* If len == 0, caller asked us to copy all of src's buffer even if it gets cut short because of dest's buffer size */
        requested_length = (len == 0U) ? *src->length : len;

        /* Get the amount of bytes that the source buffer will contain after append */
        final_length = requested_length + *dest->length;

        /* Make sure that we won't go beyond our source buffer */
        ENSURE(src->max_length >= requested_length);

        /* Make sure that the final length is not too big for our destination safe buffer */
        ENSURE(dest->max_length >= final_length);

        /* The dangerous function */
        memcpy(&dest->buffer[*dest->length], src->buffer, requested_length);

        /* Copy updated length in the source buffer */
        *dest->length = final_length;
    }

    return appended;
}


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
