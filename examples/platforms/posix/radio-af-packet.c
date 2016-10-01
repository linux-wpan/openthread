/*
 *  (C) 2016 Pengutronix, Alexander Aring <aar@pengutronix.de>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <arpa/inet.h>
#include <assert.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if_packet.h>

#include <common/code_utils.hpp>
#include "radio-posix.h"

#ifndef ETH_P_IEEE802154
#define ETH_P_IEEE802154 0x00F6
#endif

void posixRadioInit(void)
{
    struct sockaddr_ll sll;
    struct ifreq ifr;
    char *ifname;

    sSockFd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_IEEE802154));
    VerifyOrExit(sSockFd != -1, perror("socket"));

    ifname = getenv("AF_PACKET_IFNAME");
    VerifyOrExit(ifname != NULL, printf("No AF_PACKET_IFNAME as env given\n"));

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
    VerifyOrExit(ioctl(sSockFd, SIOCGIFINDEX, &ifr) == 0, perror("ioctl"));

    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifr.ifr_ifindex;
    sll.sll_protocol = htons(ETH_P_IEEE802154);
    VerifyOrExit(bind(sSockFd, (struct sockaddr *)&sll, sizeof(sll)) != -1, perror("bind"));

    sReceiveFrame.mPsdu = sReceiveMessage.mPsdu;
    sTransmitFrame.mPsdu = sTransmitMessage.mPsdu;
    sAckFrame.mPsdu = sAckMessage.mPsdu;

    return;

exit:
    close(sSockFd);
    sSockFd = -1;
    exit(EXIT_FAILURE);
}

void radioReceive(otInstance *aInstance)
{
    ssize_t rval = recv(sSockFd, sReceiveMessage.mPsdu, sizeof(sReceiveMessage.mPsdu), 0);

    if (rval < 0)
    {
        perror("recv");
        exit(EXIT_FAILURE);
    }

    sReceiveFrame.mLength = (uint8_t)(rval);

    if (sAckWait &&
        isFrameTypeAck(sReceiveFrame.mPsdu) &&
        getDsn(sReceiveFrame.mPsdu) == getDsn(sTransmitFrame.mPsdu))
    {
        sState = kStateReceive;
        sAckWait = false;

#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, isFramePending(sReceiveFrame.mPsdu), kThreadError_None);
        }
        else
#endif
        {
            otPlatRadioTransmitDone(aInstance, isFramePending(sReceiveFrame.mPsdu), kThreadError_None);
        }
    }
    else if ((sState == kStateReceive || sState == kStateTransmit))
    {
        radioProcessFrame(aInstance);
    }
}

void radioTransmit(struct RadioMessage *msg, const struct RadioPacket *pkt)
{
    ssize_t rval;

    rval = send(sSockFd, msg->mPsdu, pkt->mLength - sizeof(uint16_t), 0);
    if (rval < 0)
    {
        perror("send");
        exit(EXIT_FAILURE);
    }
}
