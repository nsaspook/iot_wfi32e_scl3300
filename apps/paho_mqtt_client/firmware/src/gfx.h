/* 
 * File:   gfx.h
 * Author: root
 *
 * Created on December 13, 2023, 6:32 PM
 */

#ifndef GFX_H
#define	GFX_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <math.h>
#include "../../firmware/lcd_drv/OledGrph.h"

#define GFX_X_MID		ccolOledMax/2
#define GFX_Y_X			81
#define GFX_Y_Y			83
#define GFX_Y_Z			85	

	void line_rot(uint32_t, uint32_t, uint32_t, uint32_t);



#ifdef	__cplusplus
}
#endif

#endif	/* GFX_H */

