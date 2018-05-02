;/**************************************************************************//**
; * @file     LoaderImages.s
; * @brief    Assembly code to include loader image
; * @copyright 2012 Nuvoton Technology Corp. All rights reserved.
; *****************************************************************************/


    AREA _image, DATA, READONLY

    EXPORT  loaderImageBase
    EXPORT  loaderImageLimit
    
    ALIGN   4
        
loaderImageBase
    INCBIN .\obj\Sample_LD.bin
loaderImageLimit

    
    END