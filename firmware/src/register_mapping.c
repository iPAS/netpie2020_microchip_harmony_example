/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#include "register_mapping.h"


static float fM1downCh[8],    fM2upCh[8],    fM2downCh[8],    fM3upCh[8],    fM3downCh[8]; 
static float fM1downChRaw[8], fM2upChRaw[8], fM2downChRaw[8], fM3upChRaw[8], fM3downChRaw[8];


st_register_t st_registers[] =
{
    {"30001", &fM1downCh[0]},  {"30081", &fM1downChRaw[0]},
    {"30003", &fM1downCh[1]},  {"30083", &fM1downChRaw[1]},
    {"30005", &fM1downCh[2]},  {"30085", &fM1downChRaw[2]},
    {"30007", &fM1downCh[3]},  {"30087", &fM1downChRaw[3]},
    {"30009", &fM1downCh[4]},  {"30089", &fM1downChRaw[4]},
    {"30011", &fM1downCh[5]},  {"30091", &fM1downChRaw[5]},
    {"30013", &fM1downCh[6]},  {"30093", &fM1downChRaw[6]},
    {"30015", &fM1downCh[7]},  {"30095", &fM1downChRaw[7]},

//    {"30017", &fM2upCh[0]},    {"30097", &fM2upChRaw[0]},
//    {"30019", &fM2upCh[1]},    {"30099", &fM2upChRaw[1]},
//    {"30021", &fM2upCh[2]},    {"30101", &fM2upChRaw[2]},
//    {"30023", &fM2upCh[3]},    {"30103", &fM2upChRaw[3]},
//    {"30025", &fM2upCh[4]},    {"30105", &fM2upChRaw[4]},
//    {"30027", &fM2upCh[5]},    {"30107", &fM2upChRaw[5]},
//    {"30029", &fM2upCh[6]},    {"30109", &fM2upChRaw[6]},
//    {"30031", &fM2upCh[7]},    {"30111", &fM2upChRaw[7]},
//
//    {"30033", &fM2downCh[0]},  {"30113", &fM2downChRaw[0]},   
//    {"30035", &fM2downCh[1]},  {"30115", &fM2downChRaw[1]},       
//    {"30037", &fM2downCh[2]},  {"30117", &fM2downChRaw[2]},       
//    {"30039", &fM2downCh[3]},  {"30119", &fM2downChRaw[3]},   
//    {"30041", &fM2downCh[4]},  {"30121", &fM2downChRaw[4]},   
//    {"30043", &fM2downCh[5]},  {"30123", &fM2downChRaw[5]},   
//    {"30045", &fM2downCh[6]},  {"30125", &fM2downChRaw[6]},   
//    {"30047", &fM2downCh[7]},  {"30127", &fM2downChRaw[7]},   
//                                                                                                     
//    {"30049", &fM3upCh[0]},    {"30129", &fM3upChRaw[0]},         
//    {"30051", &fM3upCh[1]},    {"30131", &fM3upChRaw[1]},              
//    {"30053", &fM3upCh[2]},    {"30133", &fM3upChRaw[2]},             
//    {"30055", &fM3upCh[3]},    {"30135", &fM3upChRaw[3]},             
//    {"30057", &fM3upCh[4]},    {"30137", &fM3upChRaw[4]},           
//    {"30059", &fM3upCh[5]},    {"30139", &fM3upChRaw[5]},         
//    {"30061", &fM3upCh[6]},    {"30141", &fM3upChRaw[6]},        
//    {"30063", &fM3upCh[7]},    {"30143", &fM3upChRaw[7]},            
//
//    {"30065", &fM3downCh[0]},  {"30145", &fM3downChRaw[0]},     
//    {"30067", &fM3downCh[1]},  {"30147", &fM3downChRaw[1]},     
//    {"30069", &fM3downCh[2]},  {"30149", &fM3downChRaw[2]},     
//    {"30071", &fM3downCh[3]},  {"30151", &fM3downChRaw[3]},     
//    {"30073", &fM3downCh[4]},  {"30153", &fM3downChRaw[4]},     
//    {"30075", &fM3downCh[5]},  {"30155", &fM3downChRaw[5]},     
//    {"30077", &fM3downCh[6]},  {"30157", &fM3downChRaw[6]},     
//    {"30079", &fM3downCh[7]},  {"30159", &fM3downChRaw[7]},
    
    {NULL, NULL}
};


/* *****************************************************************************
 End of File
 */
