/*
 * Copyright (c) 2003 EISLAB, Lulea University of Technology.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwBT Bluetooth stack.
 * 
 * Author: Conny Ohult <conny@sm.luth.se>
 *
 */


/* uartif.c
 *
 * Implementation of the HCI UART transport layer for Linux
 */


#include "arch/lwbtopts.h"
#include "lwbt/phybusif.h"
#include "lwbt/hci.h"
#include "lwip/debug.h"
#include "lwip/mem.h"

#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "USB/usb.h"
#include "GenericTypeDefs.h"
#include "USB/usb_host_generic.h"


#include <stdlib.h>

extern BYTE deviceAddress;

/* Initializes the physical bus interface
*/
void phybusif_init(const char * port)
{
}

err_t phybusif_reset(struct phybusif_cb *cb) 
{
	/* Init new ctrl block */
	/* Alloc new pbuf. lwIP will handle dealloc */
	if((cb->p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
		LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_reset: Could not allocate memory for pbuf\n"));
		return ERR_MEM; /* Could not allocate memory for pbuf */
	}
//	cb->q = cb->p; /* Make p the pointer to the head of the pbuf chain and q to the tail */

//	cb->tot_recvd = 0;
//	cb->recvd = 0; 

//	cb->state = W4_PACKET_TYPE;
	return ERR_OK;
}

err_t phybusif_input_event(struct phybusif_cb *cb,void * data, u16_t len) 
{
	memcpy((u8_t *)cb->p->payload, data, len);
	pbuf_header(cb->p, -HCI_EVENT_HDR_LEN);
	hci_event_input(cb->p); /* Handle incoming event */
	pbuf_free(cb->p);
	phybusif_reset(cb);
	return ERR_OK;
}

err_t phybusif_input_acl(struct phybusif_cb *cb,void * data, u16_t len) 
{
	memcpy((u8_t *)cb->p->payload, data, len);
	pbuf_header(cb->p, -HCI_ACL_HDR_LEN);
	hci_acl_input(cb->p); /* Handle incoming ACL data */
	pbuf_free(cb->p);
	phybusif_reset(cb);
	return ERR_OK;
}

void phybusif_output(struct pbuf *p, u16_t len) 
{
	static unsigned char buf[100];

//	static unsigned char *ptr;
//	unsigned char c;
	u16_t total_len;

	/* Send pbuf */
	total_len = pbuf_copy_partial(p,buf,100,1);

	if(*(u8_t*)p->payload == HCI_COMMAND_DATA_PACKET)
	{
		LWIP_DEBUGF(PHYBUSIF_DEBUG, ("sending cmd len %02x\n", total_len));
		USBHostGenericWrite0(deviceAddress,buf,total_len);
	}else if(*(u8_t*)p->payload == HCI_ACL_DATA_PACKET)
	{
		LWIP_DEBUGF(PHYBUSIF_DEBUG, ("sending acl len %02x\n", total_len));
		USBHostGenericWrite2(deviceAddress,buf,total_len);
	}
}

