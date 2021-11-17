/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _REGISTER_MAPPING_H    /* Guard against multiple inclusion */
#define _REGISTER_MAPPING_H


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


#if ! defined(NULL)
#define NULL (void *)0
#endif 

typedef     
struct 
{
    const char * const sub_topic;
    volatile float * p_value;
} st_register_t;

extern st_register_t st_registers[];


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
