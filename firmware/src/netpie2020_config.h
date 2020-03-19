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

#ifndef _NETPIE2020_CONFIG_H
#define _NETPIE2020_CONFIG_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    
/** Please define the following macros by looking in the NETPIE2020 dashboard.
 */
//#define NETPIE_DEVICE_NAME  ""
//#define NETPIE_CLIENT_ID    ""
//#define NETPIE_TOKEN        ""
//#define NETPIE_SECRET       ""

#if !defined(NETPIE_DEVICE_NAME)
#define NETPIE_DEVICE_NAME "netpie_dev" 
#warning NETPIE_DEVICE_NAME can be declared here.
#endif

#if !defined(NETPIE_CLIENT_ID)
#error NETPIE_CLIENT_ID declaration is required!
#endif
    
#if !defined(NETPIE_TOKEN)
#error NETPIE_TOKEN declaration is required!
#endif
    
#if !defined(NETPIE_SECRET)
#error NETPIE_SECRET declaration is required!
#endif

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _NETPIE2020_CONFIG_H */

/* *****************************************************************************
 End of File
 */
