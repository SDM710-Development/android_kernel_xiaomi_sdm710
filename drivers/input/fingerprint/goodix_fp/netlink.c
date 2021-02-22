/*
 * netlink interface
 *
 * Copyright (C) 2017 Goodix
 * Copyright (C) 2020 XiaoMi, Inc.
 * Copyright (C) 2021 Ivan Vecera <ivan@cera.cz>
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>

#define MAX_MSGSIZE 32

static struct sock *nl_sk;
static int pid = -1;

int gf_sendnlmsg(const char *message)
{
	struct nlmsghdr *nlh;
	struct sk_buff *skb;
	int rc;

	if (!message)
		return -EINVAL;

	if (pid < 1) {
		pr_info("cannot send msg... no receiver\n");
		return 0;
	}

	skb = nlmsg_new(MAX_MSGSIZE, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_MSGSIZE, 0);
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	strlcpy(nlmsg_data(nlh), message, MAX_MSGSIZE);

	rc = netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);
	if (rc < 0)
		pr_err("failed to send msg to userspace. rc = %d\n", rc);

	return rc;
}

static void gf_netlink_rcv(struct sk_buff *skb)
{
	struct nlmsghdr *nlh;

	/* get a reference */
	skb = skb_get(skb);

	if (skb->len >= NLMSG_HDRLEN) {
		nlh = nlmsg_hdr(skb);

		/* store pid for sending */
		pid = nlh->nlmsg_pid;

		/* ack if requested */
		if (nlh->nlmsg_flags & NLM_F_ACK)
			netlink_ack(skb, nlh, 0);

		/* release the reference */
		kfree_skb(skb);
	}
}

int netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input = gf_netlink_rcv,
	};

	nl_sk = netlink_kernel_create(&init_net, NETLINK_GOODIX_FP, &cfg);
	if (!nl_sk) {
		pr_err("goodix_fp: cannot create netlink socket\n");
		return -EIO;
	}

	return 0;
}

void netlink_exit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}
