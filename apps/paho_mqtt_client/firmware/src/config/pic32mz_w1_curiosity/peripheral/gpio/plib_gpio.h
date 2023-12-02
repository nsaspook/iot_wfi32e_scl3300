/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h UUUUUUUUU

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for GPIO_RC15 pin ***/
#define GPIO_RC15_Set()               (LATCSET = (1U<<15))
#define GPIO_RC15_Clear()             (LATCCLR = (1U<<15))
#define GPIO_RC15_Toggle()            (LATCINV= (1U<<15))
#define GPIO_RC15_OutputEnable()      (TRISCCLR = (1U<<15))
#define GPIO_RC15_InputEnable()       (TRISCSET = (1U<<15))
#define GPIO_RC15_Get()               ((PORTC >> 15) & 0x1U)
#define GPIO_RC15_PIN                  GPIO_PIN_RC15

/*** Macros for QEB2_SIG pin ***/
#define QEB2_SIG_Set()               (LATKSET = (1U<<13))
#define QEB2_SIG_Clear()             (LATKCLR = (1U<<13))
#define QEB2_SIG_Toggle()            (LATKINV= (1U<<13))
#define QEB2_SIG_OutputEnable()      (TRISKCLR = (1U<<13))
#define QEB2_SIG_InputEnable()       (TRISKSET = (1U<<13))
#define QEB2_SIG_Get()               ((PORTK >> 13) & 0x1U)
#define QEB2_SIG_PIN                  GPIO_PIN_RK13

/*** Macros for DIS_CS pin ***/
#define DIS_CS_Set()               (LATASET = (1U<<1))
#define DIS_CS_Clear()             (LATACLR = (1U<<1))
#define DIS_CS_Toggle()            (LATAINV= (1U<<1))
#define DIS_CS_OutputEnable()      (TRISACLR = (1U<<1))
#define DIS_CS_InputEnable()       (TRISASET = (1U<<1))
#define DIS_CS_Get()               ((PORTA >> 1) & 0x1U)
#define DIS_CS_PIN                  GPIO_PIN_RA1

/*** Macros for EXT_SW1 pin ***/
#define EXT_SW1_Set()               (LATASET = (1U<<4))
#define EXT_SW1_Clear()             (LATACLR = (1U<<4))
#define EXT_SW1_Toggle()            (LATAINV= (1U<<4))
#define EXT_SW1_OutputEnable()      (TRISACLR = (1U<<4))
#define EXT_SW1_InputEnable()       (TRISASET = (1U<<4))
#define EXT_SW1_Get()               ((PORTA >> 4) & 0x1U)
#define EXT_SW1_PIN                  GPIO_PIN_RA4

/*** Macros for IMU_INT1 pin ***/
#define IMU_INT1_Get()               ((PORTK >> 6) & 0x1U)
#define IMU_INT1_PIN                  GPIO_PIN_RK6

/*** Macros for TP1 pin ***/
#define TP1_Set()               (LATASET = (1U<<13))
#define TP1_Clear()             (LATACLR = (1U<<13))
#define TP1_Toggle()            (LATAINV= (1U<<13))
#define TP1_OutputEnable()      (TRISACLR = (1U<<13))
#define TP1_InputEnable()       (TRISASET = (1U<<13))
#define TP1_Get()               ((PORTA >> 13) & 0x1U)
#define TP1_PIN                  GPIO_PIN_RA13

/*** Macros for GPIO_RB12 pin ***/
#define GPIO_RB12_Set()               (LATBSET = (1U<<12))
#define GPIO_RB12_Clear()             (LATBCLR = (1U<<12))
#define GPIO_RB12_Toggle()            (LATBINV= (1U<<12))
#define GPIO_RB12_OutputEnable()      (TRISBCLR = (1U<<12))
#define GPIO_RB12_InputEnable()       (TRISBSET = (1U<<12))
#define GPIO_RB12_Get()               ((PORTB >> 12) & 0x1U)
#define GPIO_RB12_PIN                  GPIO_PIN_RB12

/*** Macros for AN1_T pin ***/
#define AN1_T_Get()               ((PORTB >> 9) & 0x1U)
#define AN1_T_PIN                  GPIO_PIN_RB9

/*** Macros for IMU_CS pin ***/
#define IMU_CS_Set()               (LATBSET = (1U<<7))
#define IMU_CS_Clear()             (LATBCLR = (1U<<7))
#define IMU_CS_Toggle()            (LATBINV= (1U<<7))
#define IMU_CS_OutputEnable()      (TRISBCLR = (1U<<7))
#define IMU_CS_InputEnable()       (TRISBSET = (1U<<7))
#define IMU_CS_Get()               ((PORTB >> 7) & 0x1U)
#define IMU_CS_PIN                  GPIO_PIN_RB7

/*** Macros for PWM1L_T pin ***/
#define PWM1L_T_Get()               ((PORTC >> 11) & 0x1U)
#define PWM1L_T_PIN                  GPIO_PIN_RC11

/*** Macros for QEA2_SIG pin ***/
#define QEA2_SIG_Set()               (LATKSET = (1U<<14))
#define QEA2_SIG_Clear()             (LATKCLR = (1U<<14))
#define QEA2_SIG_Toggle()            (LATKINV= (1U<<14))
#define QEA2_SIG_OutputEnable()      (TRISKCLR = (1U<<14))
#define QEA2_SIG_InputEnable()       (TRISKSET = (1U<<14))
#define QEA2_SIG_Get()               ((PORTK >> 14) & 0x1U)
#define QEA2_SIG_PIN                  GPIO_PIN_RK14

/*** Macros for GPIO_RC9 pin ***/
#define GPIO_RC9_Set()               (LATCSET = (1U<<9))
#define GPIO_RC9_Clear()             (LATCCLR = (1U<<9))
#define GPIO_RC9_Toggle()            (LATCINV= (1U<<9))
#define GPIO_RC9_OutputEnable()      (TRISCCLR = (1U<<9))
#define GPIO_RC9_InputEnable()       (TRISCSET = (1U<<9))
#define GPIO_RC9_Get()               ((PORTC >> 9) & 0x1U)
#define GPIO_RC9_PIN                  GPIO_PIN_RC9

/*** Macros for IMU_INT2 pin ***/
#define IMU_INT2_Set()               (LATASET = (1U<<14))
#define IMU_INT2_Clear()             (LATACLR = (1U<<14))
#define IMU_INT2_Toggle()            (LATAINV= (1U<<14))
#define IMU_INT2_OutputEnable()      (TRISACLR = (1U<<14))
#define IMU_INT2_InputEnable()       (TRISASET = (1U<<14))
#define IMU_INT2_Get()               ((PORTA >> 14) & 0x1U)
#define IMU_INT2_PIN                  GPIO_PIN_RA14

/*** Macros for VREF_IN pin ***/
#define VREF_IN_Get()               ((PORTB >> 8) & 0x1U)
#define VREF_IN_PIN                  GPIO_PIN_RB8

/*** Macros for DIS_MODE pin ***/
#define DIS_MODE_Set()               (LATCSET = (1U<<10))
#define DIS_MODE_Clear()             (LATCCLR = (1U<<10))
#define DIS_MODE_Toggle()            (LATCINV= (1U<<10))
#define DIS_MODE_OutputEnable()      (TRISCCLR = (1U<<10))
#define DIS_MODE_InputEnable()       (TRISCSET = (1U<<10))
#define DIS_MODE_Get()               ((PORTC >> 10) & 0x1U)
#define DIS_MODE_PIN                  GPIO_PIN_RC10

/*** Macros for DIS_RESET pin ***/
#define DIS_RESET_Set()               (LATCSET = (1U<<12))
#define DIS_RESET_Clear()             (LATCCLR = (1U<<12))
#define DIS_RESET_Toggle()            (LATCINV= (1U<<12))
#define DIS_RESET_OutputEnable()      (TRISCCLR = (1U<<12))
#define DIS_RESET_InputEnable()       (TRISCSET = (1U<<12))
#define DIS_RESET_Get()               ((PORTC >> 12) & 0x1U)
#define DIS_RESET_PIN                  GPIO_PIN_RC12


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/


#define    GPIO_PORT_A  (0)
#define    GPIO_PORT_B  (1)
#define    GPIO_PORT_C  (2)
#define    GPIO_PORT_K  (3)
typedef uint32_t GPIO_PORT;

typedef enum
{
    GPIO_INTERRUPT_ON_MISMATCH,
    GPIO_INTERRUPT_ON_RISING_EDGE,
    GPIO_INTERRUPT_ON_FALLING_EDGE,
    GPIO_INTERRUPT_ON_BOTH_EDGES,
}GPIO_INTERRUPT_STYLE;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/


#define     GPIO_PIN_RA0  (0U)
#define     GPIO_PIN_RA1  (1U)
#define     GPIO_PIN_RA2  (2U)
#define     GPIO_PIN_RA3  (3U)
#define     GPIO_PIN_RA4  (4U)
#define     GPIO_PIN_RA5  (5U)
#define     GPIO_PIN_RA6  (6U)
#define     GPIO_PIN_RA7  (7U)
#define     GPIO_PIN_RA8  (8U)
#define     GPIO_PIN_RA9  (9U)
#define     GPIO_PIN_RA10  (10U)
#define     GPIO_PIN_RA11  (11U)
#define     GPIO_PIN_RA12  (12U)
#define     GPIO_PIN_RA13  (13U)
#define     GPIO_PIN_RA14  (14U)
#define     GPIO_PIN_RA15  (15U)
#define     GPIO_PIN_RB0  (16U)
#define     GPIO_PIN_RB1  (17U)
#define     GPIO_PIN_RB2  (18U)
#define     GPIO_PIN_RB3  (19U)
#define     GPIO_PIN_RB4  (20U)
#define     GPIO_PIN_RB5  (21U)
#define     GPIO_PIN_RB6  (22U)
#define     GPIO_PIN_RB7  (23U)
#define     GPIO_PIN_RB8  (24U)
#define     GPIO_PIN_RB9  (25U)
#define     GPIO_PIN_RB10  (26U)
#define     GPIO_PIN_RB11  (27U)
#define     GPIO_PIN_RB12  (28U)
#define     GPIO_PIN_RB13  (29U)
#define     GPIO_PIN_RB14  (30U)
#define     GPIO_PIN_RC0  (32U)
#define     GPIO_PIN_RC1  (33U)
#define     GPIO_PIN_RC2  (34U)
#define     GPIO_PIN_RC3  (35U)
#define     GPIO_PIN_RC4  (36U)
#define     GPIO_PIN_RC5  (37U)
#define     GPIO_PIN_RC6  (38U)
#define     GPIO_PIN_RC7  (39U)
#define     GPIO_PIN_RC8  (40U)
#define     GPIO_PIN_RC9  (41U)
#define     GPIO_PIN_RC10  (42U)
#define     GPIO_PIN_RC11  (43U)
#define     GPIO_PIN_RC12  (44U)
#define     GPIO_PIN_RC13  (45U)
#define     GPIO_PIN_RC14  (46U)
#define     GPIO_PIN_RC15  (47U)
#define     GPIO_PIN_RK0  (48U)
#define     GPIO_PIN_RK1  (49U)
#define     GPIO_PIN_RK2  (50U)
#define     GPIO_PIN_RK3  (51U)
#define     GPIO_PIN_RK4  (52U)
#define     GPIO_PIN_RK5  (53U)
#define     GPIO_PIN_RK6  (54U)
#define     GPIO_PIN_RK7  (55U)
#define     GPIO_PIN_RK8  (56U)
#define     GPIO_PIN_RK9  (57U)
#define     GPIO_PIN_RK10  (58U)
#define     GPIO_PIN_RK11  (59U)
#define     GPIO_PIN_RK12  (60U)
#define     GPIO_PIN_RK13  (61U)
#define     GPIO_PIN_RK14  (62U)

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
#define    GPIO_PIN_NONE   (-1)

typedef uint32_t GPIO_PIN;


void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
	 uint32_t xvalue = (uint32_t)value;
    GPIO_PortWrite((pin>>4U), (uint32_t)(0x1U) << (pin & 0xFU), (xvalue) << (pin & 0xFU));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return ((((GPIO_PortRead((GPIO_PORT)(pin>>4U))) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (((GPIO_PortLatchRead((GPIO_PORT)(pin>>4U)) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
