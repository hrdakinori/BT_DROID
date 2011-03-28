#pragma once

#include <stdlib.h>
//#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include "uart2.h"

typedef unsigned char	uint8_t;
typedef char			int8_t;
typedef unsigned short	uint16_t;
typedef short			int16_t;
typedef unsigned long	uint32_t;
typedef long			int32_t;


//system types
typedef uint8_t		u8_t;
#define U8_F		"d"
typedef int8_t		s8_t;
#define S8_F		"d"
typedef uint16_t	u16_t;
#define U16_F		"04x"
typedef int16_t		s16_t;
#define S16_F		"04x"
typedef uint32_t	u32_t;
#define U32_F		"08x"
typedef int32_t		s32_t;
#define S32_F		"08x"
typedef int			mem_ptr_t;

#define PACK_STRUCT_STRUCT __attribute__((__packed__))

#define LWIP_PLATFORM_DIAG(x) UART2Printf x //printf x
#define LWIP_PLATFORM_ASSERT(x) UART2Printf(x) //printf(x)//; raise(SIGTRAP)

#define WORDCAST(x) (*((u8_t*)(x))|*((u8_t*)(x)+1) << 8)
#define WORDIN(x,w) do {\
					*((u8_t*)(x)) = (u8_t)((w) &0x00ff);\
					*((u8_t*)(x)+1) = (u8_t)(((w) &0xff00) >>8);\
					} while(0)
