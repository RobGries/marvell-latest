/** @file mlan_usb.c
 *
 *  @brief This file contains USB specific code
 *
 *  Copyright (C) 2008-2018, Marvell International Ltd.
 *
 *  This software file (the "File") is distributed by Marvell International
 *  Ltd. under the terms of the GNU General Public License Version 2, June 1991
 *  (the "License").  You may use, redistribute and/or modify this File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 */

/********************************************************
Change log:
    04/21/2009: initial version
********************************************************/

#include "mlan.h"
#ifdef STA_SUPPORT
#include "mlan_join.h"
#endif
#include "mlan_util.h"
#include "mlan_init.h"
#include "mlan_fw.h"
#include "mlan_main.h"

/********************************************************
			Local Variables
********************************************************/

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

/**
 *  @brief  This function downloads FW blocks to device
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param pmfw			A pointer to firmware image
 *
 *  @return             MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status
wlan_prog_fw_w_helper(IN pmlan_adapter pmadapter, IN pmlan_fw_image pmfw)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	t_u8 *firmware = pmfw->pfw_buf, *RecvBuff;
	t_u32 retries = MAX_FW_RETRY, DataLength;
	t_u32 FWSeqNum = 0, TotalBytes = 0, DnldCmd = 0;
	t_u8 *TxBuff = MNULL;
	FWData *fwdata = MNULL;
	FWSyncHeader SyncFWHeader;
	t_u8 check_winner = 1;

	ENTER();

	if (!firmware && !pcb->moal_get_fw_data) {
		PRINTM(MMSG, "No firmware image found! Terminating download\n");
		ret = MLAN_STATUS_FAILURE;
		goto fw_exit;
	}

	/* Allocate memory for transmit */
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, FW_DNLD_TX_BUF_SIZE,
			       MLAN_MEM_DEF | MLAN_MEM_DMA, (t_u8 **)&TxBuff);
	if ((ret != MLAN_STATUS_SUCCESS) || !TxBuff) {
		PRINTM(MERROR, "Could not allocate buffer for FW download\n");
		goto fw_exit;
	}
	fwdata = (FWData *)TxBuff;

	/* Allocate memory for receive */
	ret = pcb->moal_malloc(pmadapter->pmoal_handle, FW_DNLD_RX_BUF_SIZE,
			       MLAN_MEM_DEF | MLAN_MEM_DMA, &RecvBuff);
	if ((ret != MLAN_STATUS_SUCCESS) || !RecvBuff) {
		PRINTM(MERROR,
		       "Could not allocate buffer for FW download response\n");
		goto cleanup;
	}

	do {
		/* Send pseudo data to check winner status first */
		if (check_winner) {
			memset(pmadapter, &fwdata->fw_header, 0,
			       sizeof(FWHeader));
			DataLength = 0;
		} else {
			/* Copy the header of the firmware data to get the length */
			if (firmware)
				memcpy(pmadapter, &fwdata->fw_header,
				       &firmware[TotalBytes], sizeof(FWHeader));
			else
				pcb->moal_get_fw_data(pmadapter->pmoal_handle,
						      TotalBytes,
						      sizeof(FWHeader),
						      (t_u8 *)&fwdata->
						      fw_header);

			DataLength =
				wlan_le32_to_cpu(fwdata->fw_header.data_length);
			DnldCmd = wlan_le32_to_cpu(fwdata->fw_header.dnld_cmd);
			TotalBytes += sizeof(FWHeader);

	/** CMD 7 don't have data_length field */
			if (DnldCmd == FW_CMD_7)
				DataLength = 0;

			if (DataLength >
			    (FW_DNLD_TX_BUF_SIZE - sizeof(FWHeader))) {
				PRINTM(MERROR,
				       "Invalid Data Legth read from FW\n");
				ret = MLAN_STATUS_FAILURE;
				break;
			}

			/* Copy the firmware data */
			if (firmware)
				memcpy(pmadapter, fwdata->data,
				       &firmware[TotalBytes], DataLength);
			else
				pcb->moal_get_fw_data(pmadapter->pmoal_handle,
						      TotalBytes, DataLength,
						      (t_u8 *)fwdata->data);

			fwdata->seq_num = wlan_cpu_to_le32(FWSeqNum);
			TotalBytes += DataLength;
		}

		/* If the send/receive fails or CRC occurs then retry */
		while (retries) {
			mlan_buffer mbuf;
			int length = FW_DATA_XMIT_SIZE;
			retries--;

			memset(pmadapter, &mbuf, 0, sizeof(mlan_buffer));
			mbuf.pbuf = (t_u8 *)fwdata;
			mbuf.data_len = length;
			/* Send the firmware block */
			ret = pcb->moal_write_data_sync(pmadapter->pmoal_handle,
							&mbuf,
							pmadapter->tx_cmd_ep,
							MLAN_USB_BULK_MSG_TIMEOUT);
			if (ret != MLAN_STATUS_SUCCESS) {
				PRINTM(MERROR,
				       "fw_dnld: write_data failed, ret %d\n",
				       ret);
				continue;
			}

			memset(pmadapter, &mbuf, 0, sizeof(mlan_buffer));
			mbuf.pbuf = RecvBuff;
			mbuf.data_len = FW_DNLD_RX_BUF_SIZE;

			/* Receive the firmware block response */
			ret = pcb->moal_read_data_sync(pmadapter->pmoal_handle,
						       &mbuf,
						       pmadapter->rx_cmd_ep,
						       MLAN_USB_BULK_MSG_TIMEOUT);
			if (ret != MLAN_STATUS_SUCCESS) {
				PRINTM(MERROR,
				       "fw_dnld: read_data failed, ret %d\n",
				       ret);
				continue;
			}
			memcpy(pmadapter, &SyncFWHeader, RecvBuff,
			       sizeof(FWSyncHeader));
			endian_convert_syncfwheader(&SyncFWHeader);

			/* Check the first firmware block response for highest bit set */
			if (check_winner) {
				if (SyncFWHeader.cmd & 0x80000000) {
					PRINTM(MMSG,
					       "USB is not the winner 0x%x, returning success\n",
					       SyncFWHeader.cmd);
					ret = MLAN_STATUS_SUCCESS;
					goto cleanup;
				}
				PRINTM(MINFO,
				       "USB is the winner, start to download FW\n");
				check_winner = 0;
				break;
			}

			/* Check the firmware block response for CRC errors */
			if (SyncFWHeader.cmd) {
				PRINTM(MERROR,
				       "FW received Blk with CRC error 0x%x\n",
				       SyncFWHeader.cmd);
				ret = MLAN_STATUS_FAILURE;
				continue;
			}

			retries = MAX_FW_RETRY;
			break;
		}

		FWSeqNum++;
		PRINTM(MINFO, ".\n");

	} while ((DnldCmd != FW_HAS_LAST_BLOCK) && retries);

cleanup:
	PRINTM(MMSG, "fw_dnld: %d bytes downloaded\n", TotalBytes);

	if (RecvBuff)
		pcb->moal_mfree(pmadapter->pmoal_handle, RecvBuff);
	if (TxBuff)
		pcb->moal_mfree(pmadapter->pmoal_handle, TxBuff);
	if (retries) {
		ret = MLAN_STATUS_SUCCESS;
	}

fw_exit:
	LEAVE();
	return ret;
}

/**
 *  @brief Get number of packets when deaggregated
 *
 *  @param pmadapter		A pointer to mlan_adapter
 *  @param pdata			A pointer to packet data
 *  @param aggr_pkt_len		Aggregate packet length
 *
 *  @return			Number of packets
 */
static int
wlan_usb_deaggr_rx_num_pkts(pmlan_adapter pmadapter, t_u8 *pdata,
			    int aggr_pkt_len)
{
	int pkt_count = 0, pkt_len;
	RxPD *prx_pd;

	ENTER();
	while (aggr_pkt_len >= sizeof(RxPD)) {
		prx_pd = (RxPD *)pdata;
		pkt_len = wlan_le16_to_cpu(prx_pd->rx_pkt_length)
			+ wlan_le16_to_cpu(prx_pd->rx_pkt_offset);
		if (pkt_len == 0)	/* blank RxPD can be at the end */
			break;

		++pkt_count;
		if (aggr_pkt_len == pkt_len)	/* last packet has no padding */
			break;

		/* skip padding and goto next */
		if (pkt_len % pmadapter->usb_rx_deaggr.aggr_ctrl.aggr_align)
			pkt_len +=
				(pmadapter->usb_rx_deaggr.aggr_ctrl.aggr_align -
				 (pkt_len %
				  pmadapter->usb_rx_deaggr.aggr_ctrl.
				  aggr_align));
		aggr_pkt_len -= pkt_len;
		pdata += pkt_len;
	}
	LEAVE();
	return pkt_count;
}

static inline t_u32
usb_tx_aggr_pad_len(t_u32 len, usb_tx_aggr_params *pusb_tx_aggr)
{
	return (len % pusb_tx_aggr->aggr_ctrl.aggr_align) ? (len +
							     (pusb_tx_aggr->
							      aggr_ctrl.
							      aggr_align -
							      (len %
							       pusb_tx_aggr->
							       aggr_ctrl.
							       aggr_align))) :
		len;
}

/**
 *  @brief Copy pmbuf to aggregation buffer
 *
 *  @param pmadapter	Pointer to mlan_adapter structure
 *  @param pmbuf_aggr	Pointer to aggregation buffer
 *  @param pmbuf		Pointer to buffer to copy
 *  @param pusb_tx_aggr Pointer to usb_tx_aggr_params
 *
 *  @return   N/A
 */
static inline t_void
wlan_usb_tx_copy_buf_to_aggr(pmlan_adapter pmadapter,
			     pmlan_buffer pmbuf_aggr,
			     pmlan_buffer pmbuf,
			     usb_tx_aggr_params *pusb_tx_aggr)
{
	ENTER();
	pmbuf_aggr->data_len =
		usb_tx_aggr_pad_len(pmbuf_aggr->data_len, pusb_tx_aggr);
	memcpy(pmadapter,
	       pmbuf_aggr->pbuf + pmbuf_aggr->data_offset +
	       pmbuf_aggr->data_len, pmbuf->pbuf + pmbuf->data_offset,
	       pmbuf->data_len);
	pmbuf_aggr->data_len += pmbuf->data_len;
	LEAVE();
}

#define MLAN_TYPE_AGGR_DATA_V2     11
/**
 *  @brief Copy pmbuf to aggregation buffer
 *
 *  @param pmadapter	Pointer to mlan_adapter structure
 *  @param pmbuf_aggr	Pointer to aggregation buffer
 *  @param pmbuf		Pointer to buffer to copy
 *	@param last 		last packet flag
 *  @param pusb_tx_aggr Pointer to usb_tx_aggr_params
 *
 *  @return   N/A
 */
static inline t_void
wlan_usb_tx_copy_buf_to_aggr_v2(pmlan_adapter pmadapter,
				pmlan_buffer pmbuf_aggr,
				pmlan_buffer pmbuf,
				t_u8 last, usb_tx_aggr_params *pusb_tx_aggr)
{
	t_u8 *payload;
	t_u16 offset;

	ENTER();
	pmbuf_aggr->data_len =
		usb_tx_aggr_pad_len(pmbuf_aggr->data_len, pusb_tx_aggr);
	memcpy(pmadapter,
	       pmbuf_aggr->pbuf + pmbuf_aggr->data_offset +
	       pmbuf_aggr->data_len, pmbuf->pbuf + pmbuf->data_offset,
	       pmbuf->data_len);
	payload =
		pmbuf_aggr->pbuf + pmbuf_aggr->data_offset +
		pmbuf_aggr->data_len;
	if (last) {
		offset = pmbuf->data_len;
		*(t_u16 *)&payload[2] =
			wlan_cpu_to_le16(MLAN_TYPE_AGGR_DATA_V2 | 0x80);
	} else {
		offset = usb_tx_aggr_pad_len(pmbuf->data_len, pusb_tx_aggr);
		*(t_u16 *)&payload[2] =
			wlan_cpu_to_le16(MLAN_TYPE_AGGR_DATA_V2);
	}
	*(t_u16 *)&payload[0] = wlan_cpu_to_le16(offset);
	pmbuf_aggr->data_len += pmbuf->data_len;
	PRINTM(MIF_D, "offset=%d len=%d\n", offset, pmbuf->data_len);
	LEAVE();
}

/**
 *  @brief Allocate Aggregation buffer and copy pending buffers to it.
 *
 *  @param pmadapter	Pointer to mlan_adapter structure
 *  @param pusb_tx_aggr Pointer to usb_tx_aggr_params
 *
 *  @return			Aggregation buffer
 */
static inline pmlan_buffer
wlan_usb_copy_buf_to_aggr(pmlan_adapter pmadapter,
			  usb_tx_aggr_params *pusb_tx_aggr)
{
	pmlan_buffer pmbuf_aggr = MNULL;
	t_u8 i, use_count;
	pmlan_buffer pmbuf_curr, pmbuf_next;
	pmbuf_aggr =
		wlan_alloc_mlan_buffer(pmadapter, pusb_tx_aggr->aggr_len, 0,
				       MOAL_MALLOC_BUFFER);
	if (pmbuf_aggr) {
		pmbuf_curr = pusb_tx_aggr->pmbuf_aggr;
		pmbuf_aggr->bss_index = pmbuf_curr->bss_index;
		pmbuf_aggr->buf_type = pmbuf_curr->buf_type;
		pmbuf_aggr->priority = pmbuf_curr->priority;
		pmbuf_aggr->data_len = 0;
		PRINTM(MIF_D, "use_count=%d,aggr_len=%d\n",
		       pmbuf_curr->use_count, pusb_tx_aggr->aggr_len);
		use_count = pmbuf_curr->use_count;
		for (i = 0; i <= use_count; i++) {
			pmbuf_next = pmbuf_curr->pnext;
			if (pusb_tx_aggr->aggr_ctrl.aggr_mode ==
			    MLAN_USB_AGGR_MODE_LEN_V2) {
				if (i == use_count)
					wlan_usb_tx_copy_buf_to_aggr_v2
						(pmadapter, pmbuf_aggr,
						 pmbuf_curr, MTRUE,
						 pusb_tx_aggr);
				else
					wlan_usb_tx_copy_buf_to_aggr_v2
						(pmadapter, pmbuf_aggr,
						 pmbuf_curr, MFALSE,
						 pusb_tx_aggr);
			} else
				wlan_usb_tx_copy_buf_to_aggr(pmadapter,
							     pmbuf_aggr,
							     pmbuf_curr,
							     pusb_tx_aggr);
			pmbuf_curr = pmbuf_next;
		}
		DBG_HEXDUMP(MIF_D, "USB AggrTx",
			    pmbuf_aggr->pbuf + pmbuf_aggr->data_offset,
			    pmbuf_aggr->data_len);
	}
	return pmbuf_aggr;
}

/**
 *  @brief Link buffer into aggregate head buffer
 *
 *  @param pmbuf_aggr	Pointer to aggregation buffer
 *  @param pmbuf		Pointer to buffer to add to the buffer list
 *  @param pusb_tx_aggr Pointer to usb_tx_aggr_params
 */
static inline t_void
wlan_usb_tx_link_buf_to_aggr(pmlan_buffer pmbuf_aggr,
			     pmlan_buffer pmbuf,
			     usb_tx_aggr_params *pusb_tx_aggr)
{
	/* link new buf at end of list */
	pmbuf->pnext = pmbuf_aggr;
	pmbuf->pprev = pmbuf_aggr->pprev;
	pmbuf->pparent = pmbuf_aggr;
	pmbuf_aggr->pprev->pnext = pmbuf;
	pmbuf_aggr->pprev = pmbuf;
	pmbuf_aggr->use_count++;
	pusb_tx_aggr->aggr_len =
		usb_tx_aggr_pad_len(pusb_tx_aggr->aggr_len, pusb_tx_aggr);
	pusb_tx_aggr->aggr_len += pmbuf->data_len;
}

/**
 *  @brief Send aggregated buffer
 *
 *  @param pmadapter	Pointer to mlan_adapter structure
 *  @param pusb_tx_aggr Pointer to usb_tx_aggr_params
 */
static inline t_void
wlan_usb_tx_send_aggr(pmlan_adapter pmadapter, usb_tx_aggr_params *pusb_tx_aggr)
{
	mlan_status ret;
	pmlan_buffer pmbuf_aggr = pusb_tx_aggr->pmbuf_aggr;
	ENTER();
	if (!pusb_tx_aggr->pmbuf_aggr) {
		LEAVE();
		return;
	}

	if (pusb_tx_aggr->pmbuf_aggr->use_count) {
		pmbuf_aggr = wlan_usb_copy_buf_to_aggr(pmadapter, pusb_tx_aggr);
		/* allocate new buffer for aggregation if not exist */
		if (!pmbuf_aggr) {
			PRINTM(MERROR,
			       "Error allocating [usb_tx] aggr mlan_buffer.\n");
			pmadapter->dbg.num_tx_host_to_card_failure +=
				pusb_tx_aggr->pmbuf_aggr->use_count;
			wlan_write_data_complete(pmadapter,
						 pusb_tx_aggr->pmbuf_aggr,
						 MLAN_STATUS_FAILURE);
			pusb_tx_aggr->pmbuf_aggr = MNULL;
			pusb_tx_aggr->aggr_len = 0;
			LEAVE();
			return;
		} else {
			wlan_write_data_complete(pmadapter,
						 pusb_tx_aggr->pmbuf_aggr,
						 MLAN_STATUS_SUCCESS);
			pusb_tx_aggr->pmbuf_aggr = MNULL;
			pusb_tx_aggr->aggr_len = 0;
		}
	} else if (pusb_tx_aggr->aggr_ctrl.aggr_mode ==
		   MLAN_USB_AGGR_MODE_LEN_V2) {
		t_u8 *payload = pmbuf_aggr->pbuf + pmbuf_aggr->data_offset;
		*(t_u16 *)&payload[0] = wlan_cpu_to_le16(pmbuf_aggr->data_len);
		*(t_u16 *)&payload[2] =
			wlan_cpu_to_le16(MLAN_TYPE_AGGR_DATA_V2 | 0x80);
		PRINTM(MIF_D, "USB Send single packet len=%d\n",
		       pmbuf_aggr->data_len);
		DBG_HEXDUMP(MIF_D, "USB Tx",
			    pmbuf_aggr->pbuf + pmbuf_aggr->data_offset,
			    pmbuf_aggr->data_len);
	}

	if (pmbuf_aggr && pmbuf_aggr->data_len) {
		wlan_update_port_status(pmadapter, pusb_tx_aggr->port, MTRUE);
		ret = pmadapter->callbacks.moal_write_data_async(pmadapter->
								 pmoal_handle,
								 pmbuf_aggr,
								 pusb_tx_aggr->
								 port);
		switch (ret) {
		case MLAN_STATUS_PRESOURCE:
			pmadapter->data_sent = wlan_usb_data_sent(pmadapter);
			PRINTM(MINFO, "MLAN_STATUS_PRESOURCE is returned\n");
			break;
		case MLAN_STATUS_RESOURCE:
			/* Shouldn't reach here due to next condition. */
			/* TODO: (maybe) How to requeue the aggregate? */
			/* It may occur when the pending tx urbs reach the high mark */
			/* Thus, block further pkts for a bit */
			pmadapter->data_sent = wlan_usb_data_sent(pmadapter);
			PRINTM(MERROR,
			       "Error: moal_write_data_async failed: 0x%X\n",
			       ret);
			pmadapter->dbg.num_tx_host_to_card_failure++;
			pmbuf_aggr->status_code = MLAN_ERROR_DATA_TX_FAIL;
			wlan_write_data_complete(pmadapter, pmbuf_aggr, ret);
			break;
		case MLAN_STATUS_FAILURE:
			wlan_update_port_status(pmadapter, pusb_tx_aggr->port,
						MFALSE);
			PRINTM(MERROR,
			       "Error: moal_write_data_async failed: 0x%X\n",
			       ret);
			pmadapter->dbg.num_tx_host_to_card_failure++;
			pmbuf_aggr->status_code = MLAN_ERROR_DATA_TX_FAIL;
			wlan_write_data_complete(pmadapter, pmbuf_aggr, ret);
			break;
		case MLAN_STATUS_PENDING:
			wlan_update_port_status(pmadapter, pusb_tx_aggr->port,
						MFALSE);
			break;
		case MLAN_STATUS_SUCCESS:
			wlan_write_data_complete(pmadapter, pmbuf_aggr, ret);
			break;
		default:
			break;
		}

		/* aggr_buf now sent to bus, prevent others from using it */
		pusb_tx_aggr->pmbuf_aggr = MNULL;
		pusb_tx_aggr->aggr_len = 0;
	}
	LEAVE();
}

/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief  This function downloads firmware to card
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param pmfw			A pointer to firmware image
 *
 *  @return		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_dnld_fw(IN pmlan_adapter pmadapter, IN pmlan_fw_image pmfw)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;

	ENTER();

	ret = wlan_prog_fw_w_helper(pmadapter, pmfw);
	if (ret != MLAN_STATUS_SUCCESS) {
		LEAVE();
		return MLAN_STATUS_FAILURE;
	}

	LEAVE();
	return ret;
}

/**
 *  @brief  This function deaggregates USB RX Data Packet from device
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param pmbuf		A pointer to the received buffer
 *
 *  @return		MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
mlan_status
wlan_usb_deaggr_rx_pkt(IN pmlan_adapter pmadapter, IN pmlan_buffer pmbuf)
{
	const t_u8 zero_rx_pd[sizeof(RxPD)] = { 0 };
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u32 curr_pkt_len;
	RxPD *prx_pd;
	t_u8 *pdata;
	t_s32 aggr_len;
	pmlan_buffer pdeaggr_buf;

	ENTER();

	pdata = pmbuf->pbuf + pmbuf->data_offset;
	prx_pd = (RxPD *)pdata;
	curr_pkt_len = wlan_le16_to_cpu(prx_pd->rx_pkt_length)
		+ wlan_le16_to_cpu(prx_pd->rx_pkt_offset);
	/* if non-aggregate, just send through, don’t process here */
	aggr_len = pmbuf->data_len;
	if ((aggr_len == curr_pkt_len)
	    || (wlan_usb_deaggr_rx_num_pkts(pmadapter, pdata, aggr_len) == 1)
	    || (pmadapter->usb_rx_deaggr.aggr_ctrl.enable != MTRUE)) {
		ret = wlan_handle_rx_packet(pmadapter, pmbuf);
		LEAVE();
		return ret;
	}

	while (aggr_len >= sizeof(RxPD)) {
		/* check for (all-zeroes) termination RxPD */
		if (!memcmp(pmadapter, pdata, zero_rx_pd, sizeof(RxPD))) {
			break;
		}

		/* make new buffer and copy packet to it (including RxPD).
		 * Also, reserve headroom so that there must have space
		 * to change RxPD to TxPD for bridge packet in uAP mode */
		pdeaggr_buf = wlan_alloc_mlan_buffer(pmadapter, curr_pkt_len,
						     MLAN_RX_HEADER_LEN,
						     MOAL_ALLOC_MLAN_BUFFER);
		if (pdeaggr_buf == MNULL) {
			PRINTM(MERROR,
			       "Error allocating [usb_rx] deaggr mlan_buffer\n");
			ret = MLAN_STATUS_FAILURE;
			break;
		}
		pdeaggr_buf->bss_index = pmbuf->bss_index;
		pdeaggr_buf->buf_type = pmbuf->buf_type;
		pdeaggr_buf->data_len = curr_pkt_len;
		pdeaggr_buf->in_ts_sec = pmbuf->in_ts_sec;
		pdeaggr_buf->in_ts_usec = pmbuf->in_ts_usec;
		pdeaggr_buf->priority = pmbuf->priority;
		memcpy(pmadapter, pdeaggr_buf->pbuf + pdeaggr_buf->data_offset,
		       pdata, curr_pkt_len);

		/* send new packet to processing */
		ret = wlan_handle_rx_packet(pmadapter, pdeaggr_buf);
		if (ret == MLAN_STATUS_FAILURE) {
			break;
		}
		/* last block has no padding bytes */
		if (aggr_len == curr_pkt_len) {
			break;
		}

		/* round up to next block boundary */
		if (curr_pkt_len %
		    pmadapter->usb_rx_deaggr.aggr_ctrl.aggr_align)
			curr_pkt_len +=
				(pmadapter->usb_rx_deaggr.aggr_ctrl.aggr_align -
				 (curr_pkt_len %
				  pmadapter->usb_rx_deaggr.aggr_ctrl.
				  aggr_align));
		/* point to next packet */
		aggr_len -= curr_pkt_len;
		pdata += curr_pkt_len;
		prx_pd = (RxPD *)pdata;
		curr_pkt_len = wlan_le16_to_cpu(prx_pd->rx_pkt_length)
			+ wlan_le16_to_cpu(prx_pd->rx_pkt_offset);
	}

	/* free original pmbuf (since not sent for processing) */
	pmadapter->callbacks.moal_recv_complete(pmadapter->pmoal_handle,
						pmbuf, pmadapter->rx_data_ep,
						ret);
	LEAVE();
	return ret;
}

/**
 *  @brief This function restore tx_pause flag
 *
 *  @param pmadapter    A pointer to mlan_adapter structure
 *  @param pusb_tx_aggr A pointer to usb_tx_aggr_params
 *
 *  @return             MTRUE/MFALSE
 */
t_u8
wlan_is_port_tx_paused(pmlan_adapter pmadapter,
		       usb_tx_aggr_params *pusb_tx_aggr)
{
	mlan_private *pmpriv = MNULL;
	t_u8 i;
	t_u8 ret = MFALSE;
	for (i = 0; i < pmadapter->priv_num; i++) {
		pmpriv = pmadapter->priv[i];
		if (pmpriv && pmpriv->tx_pause &&
		    (pmpriv->port == pusb_tx_aggr->port)) {
			ret = MTRUE;
			break;
		}
	}
	return ret;
}

/**
 *  @brief This function handles the timeout of usb tx aggregation.
 *  It will send the aggregate buffer being held.
 *
 *  @param function_context   A pointer to function_context
 *  @return 	   N/A
 */
t_void
wlan_usb_tx_aggr_timeout_func(t_void *function_context)
{

	usb_tx_aggr_params *pusb_tx_aggr =
		(usb_tx_aggr_params *)function_context;
	t_u8 port_index = 0;
	pmlan_adapter pmadapter = (mlan_adapter *)pusb_tx_aggr->phandle;
	pmlan_callbacks pcb = &pmadapter->callbacks;

	ENTER();
	pcb->moal_spin_lock(pmadapter->pmoal_handle, pusb_tx_aggr->paggr_lock);
	pusb_tx_aggr->aggr_hold_timer_is_set = MFALSE;
	port_index = wlan_get_port_index(pmadapter, pusb_tx_aggr->port);
	if (pusb_tx_aggr->pmbuf_aggr &&
	    wlan_is_port_ready(pmadapter, port_index)
	    && !wlan_is_port_tx_paused(pmadapter, pusb_tx_aggr)
		)
		wlan_usb_tx_send_aggr(pmadapter, pusb_tx_aggr);
	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pusb_tx_aggr->paggr_lock);
	LEAVE();
}

/**
 *  @brief  This function resets USB Tx Aggregation buffers
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *
 *  @return 	N/A
 */
inline t_void
wlan_reset_usb_tx_aggr(IN pmlan_adapter pmadapter)
{
	t_s32 i = 0;
	pmlan_callbacks pcb = &pmadapter->callbacks;
	ENTER();
	for (i = 0; i < MAX_USB_TX_PORT_NUM; i++) {
		pcb->moal_spin_lock(pmadapter->pmoal_handle,
				    pmadapter->usb_tx_aggr[i].paggr_lock);
		if (pmadapter->usb_tx_aggr[i].aggr_hold_timer_is_set) {
			pcb->moal_stop_timer(pmadapter->pmoal_handle,
					     pmadapter->usb_tx_aggr[i].
					     paggr_hold_timer);
			pmadapter->usb_tx_aggr[i].aggr_hold_timer_is_set =
				MFALSE;
		}
		if (pmadapter->usb_tx_aggr[i].aggr_ctrl.enable &&
		    pmadapter->usb_tx_aggr[i].pmbuf_aggr != MNULL) {
			wlan_write_data_complete(pmadapter, pmadapter->usb_tx_aggr[i].pmbuf_aggr, MLAN_STATUS_FAILURE);	/* did not get sent */
			pmadapter->usb_tx_aggr[i].pmbuf_aggr = MNULL;
			pmadapter->usb_tx_aggr[i].aggr_len = 0;
		}
		pcb->moal_spin_unlock(pmadapter->pmoal_handle,
				      pmadapter->usb_tx_aggr[i].paggr_lock);
	}
	LEAVE();
}

/**
 *  @brief  This function get usb_tx_aggr_params
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param port		    port for TX
 *
 *  @return	            A pointer to usb_tx_aggr_params
 */
inline usb_tx_aggr_params *
wlan_get_usb_tx_aggr_params(IN pmlan_adapter pmadapter, IN t_u32 port)
{
	int i;
	ENTER();
	for (i = 0; i < MAX_USB_TX_PORT_NUM; i++) {
		if (pmadapter->usb_tx_aggr[i].aggr_ctrl.enable &&
		    pmadapter->usb_tx_aggr[i].port == port)
			return &pmadapter->usb_tx_aggr[i];
	}
	LEAVE();
	return MNULL;
}

/**
 *  @brief  This function aggregates USB TX Data Packet to send to device
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param pmbuf		A pointer to the transmit buffer
 *  @param tx_param 	A pointer to mlan_tx_param
 *  @param pusb_tx_aggr A pointer to usb_tx_aggr_params
 *
 *  @return		MLAN_STATUS_PENDING or MLAN_STATUS_FAILURE
 */
/*
 *  Non Scatter-Gather code creates a new large buffer where each incoming
 *     buffer's data contents are copied to (aligned to USB boundaries).
 *  The individual buffers are ALSO linked to the large buffer,
 *    in order to handle complete AFTER the aggregate is sent.
 *  pmbuf_aggr->data_len is used to keep track of bytes aggregated so far.
 */
mlan_status
wlan_usb_host_to_card_aggr(IN pmlan_adapter pmadapter,
			   IN pmlan_buffer pmbuf,
			   IN mlan_tx_param *tx_param,
			   IN usb_tx_aggr_params *pusb_tx_aggr)
{
	pmlan_callbacks pcb = &pmadapter->callbacks;
	pmlan_buffer pmbuf_aggr;
	mlan_status ret = MLAN_STATUS_PENDING;
	t_u32 next_pkt_len = (tx_param) ? tx_param->next_pkt_len : 0;
	t_u32 aggr_len_counter = 0;
	/* indicators */
	t_u8 f_precopy_cur_buf = 0;
	t_u8 f_send_aggr_buf = 0;
	t_u8 f_postcopy_cur_buf = 0;
	t_u32 max_aggr_size = 0, max_aggr_num = 0;

	ENTER();

	pcb->moal_spin_lock(pmadapter->pmoal_handle, pusb_tx_aggr->paggr_lock);

	/* stop timer while we process */
	if (pusb_tx_aggr->aggr_hold_timer_is_set) {
		pcb->moal_stop_timer(pmadapter->pmoal_handle,
				     pusb_tx_aggr->paggr_hold_timer);
		pusb_tx_aggr->aggr_hold_timer_is_set = MFALSE;
	}

	pmbuf_aggr = pusb_tx_aggr->pmbuf_aggr;

	if (pusb_tx_aggr->aggr_ctrl.aggr_tmo == MLAN_USB_TX_AGGR_TIMEOUT_DYN) {
		if (!pmbuf_aggr) {
			/* Start aggr from min timeout value in micro sec */
			pusb_tx_aggr->hold_timeout_msec =
				MLAN_USB_TX_MIN_AGGR_TIMEOUT;
		} else {
			/* Increase timeout in milisecond if pkts are consecutive */
			if (pusb_tx_aggr->hold_timeout_msec <
			    MLAN_USB_TX_MAX_AGGR_TIMEOUT)
				pusb_tx_aggr->hold_timeout_msec++;
		}
	} else {
		if (pusb_tx_aggr->aggr_ctrl.aggr_tmo)
			pusb_tx_aggr->hold_timeout_msec =
				pusb_tx_aggr->aggr_ctrl.aggr_tmo / 1000;
	}

	max_aggr_size = max_aggr_num = pusb_tx_aggr->aggr_ctrl.aggr_max;
	if (pusb_tx_aggr->aggr_ctrl.aggr_mode == MLAN_USB_AGGR_MODE_NUM) {
		max_aggr_size *=
			MAX(MLAN_USB_MAX_PKT_SIZE,
			    pusb_tx_aggr->aggr_ctrl.aggr_align);
	}
	if (pusb_tx_aggr->aggr_ctrl.aggr_mode == MLAN_USB_AGGR_MODE_LEN)
		max_aggr_num /= pusb_tx_aggr->aggr_ctrl.aggr_align;
	else if (pusb_tx_aggr->aggr_ctrl.aggr_mode == MLAN_USB_AGGR_MODE_LEN_V2)
		max_aggr_num = MLAN_USB_TX_AGGR_MAX_NUM;
	if (!pmbuf_aggr) {
		/* use this buf to start linked list, that's it */
		pmbuf->pnext = pmbuf->pprev = pmbuf;
		pmbuf_aggr = pmbuf;
		pusb_tx_aggr->pmbuf_aggr = pmbuf_aggr;
		pusb_tx_aggr->aggr_len = pmbuf->data_len;
		pmbuf->flags |= MLAN_BUF_FLAG_USB_TX_AGGR;

	} else {
		/* DECIDE what to do */
		aggr_len_counter =
			usb_tx_aggr_pad_len(pusb_tx_aggr->aggr_len,
					    pusb_tx_aggr);

		if ((aggr_len_counter + pmbuf->data_len) < max_aggr_size) {
			f_precopy_cur_buf = 1;	/* can fit current packet in aggr */
			if (next_pkt_len) {
				aggr_len_counter +=
					usb_tx_aggr_pad_len(pmbuf->data_len,
							    pusb_tx_aggr);
				if ((aggr_len_counter + next_pkt_len) >=
				    max_aggr_size)
					f_send_aggr_buf = 1;	/* can't fit next packet, send now */
			}
		} else {
			/* can't fit current packet */
			if (pusb_tx_aggr->aggr_len)
				f_send_aggr_buf = 1;	/* send aggr first */
			f_postcopy_cur_buf = 1;	/* then copy into new aggr_buf */
		}
	}

	/* For zero timeout and zero next packet length send pkt now */
	if (!pusb_tx_aggr->aggr_ctrl.aggr_tmo && !next_pkt_len)
		f_send_aggr_buf = 1;

	/* PERFORM ACTIONS as decided */
	if (f_precopy_cur_buf) {
		PRINTM(MIF_D, "%s: Precopy current buffer.\n", __FUNCTION__);
		wlan_usb_tx_link_buf_to_aggr(pmbuf_aggr, pmbuf, pusb_tx_aggr);
	}
	if (pmbuf_aggr->use_count + 1 >= max_aggr_num)
		f_send_aggr_buf = 1;

	if (pmbuf->flags & MLAN_BUF_FLAG_NULL_PKT
	    || pmbuf->flags & MLAN_BUF_FLAG_TCP_ACK)
		f_send_aggr_buf = 1;

	if (f_send_aggr_buf) {
		PRINTM(MIF_D, "%s: Send aggregate buffer.\n", __FUNCTION__);
		wlan_usb_tx_send_aggr(pmadapter, pusb_tx_aggr);
		pmbuf_aggr = pusb_tx_aggr->pmbuf_aggr;	/* update ptr */
	}

	if (f_postcopy_cur_buf) {
		PRINTM(MIF_D, "%s: Postcopy current buffer.\n", __FUNCTION__);
		if (!pmbuf_aggr) {	/* this is possible if just sent (above) */
			/* use this buf to start linked list */
			pmbuf->pnext = pmbuf->pprev = pmbuf;
			pmbuf_aggr = pmbuf;
			pusb_tx_aggr->pmbuf_aggr = pmbuf_aggr;
			pusb_tx_aggr->aggr_len = pmbuf->data_len;
			pmbuf->flags |= MLAN_BUF_FLAG_USB_TX_AGGR;
		}
	}
	/* (re)start timer if there is something in the aggregation buffer */
	if (pmbuf_aggr && pmbuf_aggr->data_len) {
		if (pusb_tx_aggr->aggr_ctrl.aggr_tmo) {
			pcb->moal_start_timer(pmadapter->pmoal_handle,
					      pusb_tx_aggr->paggr_hold_timer,
					      MFALSE,
					      pusb_tx_aggr->hold_timeout_msec);
			pusb_tx_aggr->aggr_hold_timer_is_set = MTRUE;
		}
	}

	pcb->moal_spin_unlock(pmadapter->pmoal_handle,
			      pusb_tx_aggr->paggr_lock);
	LEAVE();
	return ret;
}

/**
 *  @brief  This function used to check if any USB port still available
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *
 *  @return		MTRUE--non of the port is available.
 *              MFALSE -- still have port available.
 */
inline t_u8
wlan_usb_data_sent(IN pmlan_adapter pmadapter)
{
	int i;
	for (i = 0; i < MAX_USB_TX_PORT_NUM; i++) {
		if (pmadapter->usb_port_status[i] == MFALSE)
			return MFALSE;
	}
	return MTRUE;
}

/**
 *  @brief  This function used to check if specific port is ready
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param port_index   port index;
 *
 *  @return		MTRUE -- port is ready.
 *              MFALSE -- port is busy.
 */
inline t_u8
wlan_is_port_ready(IN pmlan_adapter pmadapter, t_u32 port_index)
{
	return (pmadapter->usb_port_status[port_index]) ? MFALSE : MTRUE;
}

/**
 *  @brief  This function return port index
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param port         USB port
 *  @return		port index
 *
 */
inline t_u8
wlan_get_port_index(pmlan_adapter pmadapter, t_u32 port)
{
	t_u8 i;
	for (i = 0; i < MAX_USB_TX_PORT_NUM; i++) {
		if (port == pmadapter->usb_tx_ports[i]) {
			return i;
		}
	}
	return 0;
}

/**
 *  @brief  This function update the port status
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *  @param port         USB port
 *  @param status       port status
 *
 *  @return		N/A
 */
inline void
wlan_update_port_status(pmlan_adapter pmadapter, t_u32 port, t_u8 status)
{
	int i;
	for (i = 0; i < MAX_USB_TX_PORT_NUM; i++) {
		if (port == pmadapter->usb_tx_ports[i]) {
			pmadapter->usb_port_status[i] = status;
			break;
		}
	}
	return;
}

/**
 *  @brief  This function resync the USB tx port
 *
 *  @param pmadapter	A pointer to mlan_adapter
 *
 *  @return		N/A
 */
void
wlan_resync_usb_port(pmlan_adapter pmadapter)
{
	t_u32 active_port = pmadapter->usb_tx_ports[0];
	int i;
	/* MC is enabled */
	if (pmadapter->mc_status) {
		for (i = 0; i < MIN(pmadapter->priv_num, MLAN_MAX_BSS_NUM); i++) {
			if (pmadapter->priv[i]) {
				if (((GET_BSS_ROLE(pmadapter->priv[i]) ==
				      MLAN_BSS_ROLE_UAP)
				     && !pmadapter->priv[i]->uap_bss_started) ||
				    ((GET_BSS_ROLE(pmadapter->priv[i]) ==
				      MLAN_BSS_ROLE_STA)
				     && !pmadapter->priv[i]->media_connected)) {
					PRINTM(MINFO,
					       "Set deactive interface to default EP\n");
					pmadapter->priv[i]->port =
						pmadapter->usb_tx_ports[0];
					pmadapter->priv[i]->port_index = 0;
				}
			}
		}
	/** Enable all the ports */
		for (i = 0; i < MAX_USB_TX_PORT_NUM; i++)
			pmadapter->usb_port_status[i] = MFALSE;
	} else {
		/* Get active port from connected interface */
		for (i = 0; i < MIN(pmadapter->priv_num, MLAN_MAX_BSS_NUM); i++) {
			if (pmadapter->priv[i]) {
				if (((GET_BSS_ROLE(pmadapter->priv[i]) ==
				      MLAN_BSS_ROLE_UAP)
				     && pmadapter->priv[i]->uap_bss_started) ||
				    ((GET_BSS_ROLE(pmadapter->priv[i]) ==
				      MLAN_BSS_ROLE_STA)
				     && pmadapter->priv[i]->media_connected)) {
					active_port = pmadapter->priv[i]->port;
					PRINTM(MEVENT, "active port=%d\n",
					       active_port);
					break;
				}
			}
		}
	/** set all the interface to the same port */
		for (i = 0; i < MIN(pmadapter->priv_num, MLAN_MAX_BSS_NUM); i++) {
			if (pmadapter->priv[i]) {
				pmadapter->priv[i]->port = active_port;
				pmadapter->priv[i]->port_index =
					wlan_get_port_index(pmadapter,
							    active_port);
			}
		}
		for (i = 0; i < MAX_USB_TX_PORT_NUM; i++) {
			if (active_port == pmadapter->usb_tx_ports[i])
				pmadapter->usb_port_status[i] = MFALSE;
			else
				pmadapter->usb_port_status[i] = MTRUE;
		}
	}
	return;
}
