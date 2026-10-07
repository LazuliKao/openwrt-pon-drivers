// SPDX-License-Identifier: GPL-2.0-only
/*
 * H3C HM2004-DU Micro-OLT Virtual Network Interfaces (molt_omci & molt_ploam)
 */

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include "fmcs.h"

static netdev_tx_t fmcs_omci_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fmcs_priv *priv = *(struct fmcs_priv **)netdev_priv(dev);
	struct net_device *dst_dev;

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	dst_dev = dev_get_by_name(&init_net, "lan1");
	if (!dst_dev)
		dst_dev = dev_get_by_name(&init_net, "eth1");

	if (dst_dev) {
		struct sk_buff *nskb = skb_copy(skb, GFP_ATOMIC);
		if (nskb) {
			nskb->dev = dst_dev;
			dev_queue_xmit(nskb);
		}
		dev_put(dst_dev);
	}

	(void)priv;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static netdev_tx_t fmcs_ploam_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct fmcs_priv *priv = *(struct fmcs_priv **)netdev_priv(dev);

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	if (priv) {
		mutex_lock(&priv->lock);
		fmcs_spi_send_ploam(priv, skb->data, skb->len);
		mutex_unlock(&priv->lock);
	}

	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops fmcs_omci_ops = {
	.ndo_start_xmit = fmcs_omci_xmit,
};

static const struct net_device_ops fmcs_ploam_ops = {
	.ndo_start_xmit = fmcs_ploam_xmit,
};

void fmcs_rx_omci_frame(struct fmcs_priv *priv, const u8 *data, size_t len)
{
	struct sk_buff *skb;

	if (!priv->omci_dev || !(priv->omci_dev->flags & IFF_UP))
		return;

	skb = netdev_alloc_skb(priv->omci_dev, len + 2);
	if (!skb)
		return;

	skb_reserve(skb, 2);
	skb_put_data(skb, data, len);
	skb->protocol = eth_type_trans(skb, priv->omci_dev);

	priv->omci_dev->stats.rx_packets++;
	priv->omci_dev->stats.rx_bytes += len;
	netif_rx(skb);
}

void fmcs_rx_ploam_frame(struct fmcs_priv *priv, const u8 *data, size_t len)
{
	struct sk_buff *skb;

	if (!priv->ploam_dev || !(priv->ploam_dev->flags & IFF_UP))
		return;

	skb = netdev_alloc_skb(priv->ploam_dev, len + 2);
	if (!skb)
		return;

	skb_reserve(skb, 2);
	skb_put_data(skb, data, len);
	skb->protocol = eth_type_trans(skb, priv->ploam_dev);

	priv->ploam_dev->stats.rx_packets++;
	priv->ploam_dev->stats.rx_bytes += len;
	netif_rx(skb);
}

int fmcs_netdevs_init(struct fmcs_priv *priv)
{
	int ret;

	/* Create molt_omci */
	priv->omci_dev = alloc_netdev(sizeof(struct fmcs_priv *), "molt_omci",
				      NET_NAME_UNKNOWN, ether_setup);
	if (!priv->omci_dev)
		return -ENOMEM;

	priv->omci_dev->netdev_ops = &fmcs_omci_ops;
	*(struct fmcs_priv **)netdev_priv(priv->omci_dev) = priv;
	eth_hw_addr_random(priv->omci_dev);

	ret = register_netdev(priv->omci_dev);
	if (ret) {
		free_netdev(priv->omci_dev);
		priv->omci_dev = NULL;
		return ret;
	}

	/* Create molt_ploam */
	priv->ploam_dev = alloc_netdev(sizeof(struct fmcs_priv *), "molt_ploam",
				       NET_NAME_UNKNOWN, ether_setup);
	if (!priv->ploam_dev) {
		unregister_netdev(priv->omci_dev);
		free_netdev(priv->omci_dev);
		priv->omci_dev = NULL;
		return -ENOMEM;
	}

	priv->ploam_dev->netdev_ops = &fmcs_ploam_ops;
	*(struct fmcs_priv **)netdev_priv(priv->ploam_dev) = priv;
	eth_hw_addr_random(priv->ploam_dev);

	ret = register_netdev(priv->ploam_dev);
	if (ret) {
		free_netdev(priv->ploam_dev);
		priv->ploam_dev = NULL;
		unregister_netdev(priv->omci_dev);
		free_netdev(priv->omci_dev);
		priv->omci_dev = NULL;
		return ret;
	}

	return 0;
}

void fmcs_netdevs_exit(struct fmcs_priv *priv)
{
	if (priv->ploam_dev) {
		unregister_netdev(priv->ploam_dev);
		free_netdev(priv->ploam_dev);
		priv->ploam_dev = NULL;
	}
	if (priv->omci_dev) {
		unregister_netdev(priv->omci_dev);
		free_netdev(priv->omci_dev);
		priv->omci_dev = NULL;
	}
}
