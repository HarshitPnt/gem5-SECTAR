/*
 * Copyright (c) 2020 Advanced Micro Devices, Inc.
 * Copyright (c) 2008 Princeton University
 * Copyright (c) 2016 Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/ruby/network/garnet/GarnetNetwork.hh"

#include <cassert>

#include "base/cast.hh"
#include "base/compiler.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/common/NetDest.hh"
#include "mem/ruby/network/MessageBuffer.hh"
#include "mem/ruby/network/garnet/CommonTypes.hh"
#include "mem/ruby/network/garnet/CreditLink.hh"
#include "mem/ruby/network/garnet/GarnetLink.hh"
#include "mem/ruby/network/garnet/NetworkInterface.hh"
#include "mem/ruby/network/garnet/NetworkLink.hh"
#include "mem/ruby/network/garnet/Router.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

namespace garnet
{

/*
 * GarnetNetwork sets up the routers and links and collects stats.
 * Default parameters (GarnetNetwork.py) can be overwritten from command line
 * (see configs/network/Network.py)
 */

int _router_id_to_index(int router_id, int num_cols, int num_rows)
{
    // 0 starts from last row
    int row = num_rows - router_id / num_cols - 1;
    int col = router_id % num_cols;
    // std::cout << "Router ID: " << router_id << " Row: " << row
    //           << " Col: " << col << " index " << row * num_cols + col
    //           << std::endl;
    return row * num_cols + col;
}

GarnetNetwork::GarnetNetwork(const Params &p) : Network(p)
{
    m_num_rows = p.num_rows;
    m_ni_flit_size = p.ni_flit_size;
    m_max_vcs_per_vnet = 0;
    m_buffers_per_data_vc = p.buffers_per_data_vc;
    m_buffers_per_ctrl_vc = p.buffers_per_ctrl_vc;
    m_routing_algorithm = p.routing_algorithm;
    m_num_cols = p.routers.size() / m_num_rows;
    m_trojan_router_id = p.trojan_router_id;
    // extract the trojan router ids from the list of ids as string
    // m_trojan_router_ids and convert to int
    std::vector<int> trojan_router_ids;
    std::istringstream iss(p.trojan_router_id);
    std::string id;
    while (std::getline(iss, id, ' '))
    {
        trojan_router_ids.push_back(std::stoi(id));
        std::cout << "Trojan Router ID: " << id << std::endl;
    }

    m_enable_fault_model = p.enable_fault_model;
    if (m_enable_fault_model)
        fault_model = p.fault_model;

    m_vnet_type.resize(m_virtual_networks);

    for (int i = 0; i < m_virtual_networks; i++)
    {
        if (m_vnet_type_names[i] == "response")
            m_vnet_type[i] = DATA_VNET_; // carries data (and ctrl) packets
        else
            m_vnet_type[i] = CTRL_VNET_; // carries only ctrl packets
    }

    // record the routers
    for (std::vector<BasicRouter *>::const_iterator i = p.routers.begin();
         i != p.routers.end(); ++i)
    {
        Router *router = safe_cast<Router *>(*i);
        m_routers.push_back(router);

        // initialize the router's network pointers
        router->init_net_ptr(this);
        if (std::find(trojan_router_ids.begin(), trojan_router_ids.end(),
                      router->get_id()) != trojan_router_ids.end())
        {
            router->set_trojan(true);
            std::cout << "Router " << router->get_id() << " is a trojan\n"
                      << std::endl;
        }
        else
        {
            router->set_trojan(false);
        }
        std::cout << "Router ID: " << router->get_id() << std::endl;
    }

    // set alert flags for propagation routers
    if (trojan_router_ids.size() > 0)
    {
        for (std::vector<int>::const_iterator i = trojan_router_ids.begin();
             i != trojan_router_ids.end(); ++i)
        {
            int trojan_router_id = *i;
            int trojan_index =
                _router_id_to_index(trojan_router_id, m_num_cols, m_num_rows);
            if (trojan_index / m_num_cols >= 1 &&
                trojan_index % m_num_cols >= 1)
            {
                // int index = _router_id_to_index(
                //     trojan_router_id + m_num_cols - 1, m_num_cols,
                //     m_num_rows);
                int index = trojan_router_id + m_num_cols - 1;
                m_routers[index]->set_alert_flag(2);
                std::cout << "Router " << m_routers[index]->get_id()
                          << " is a propagation router\n"
                          << std::endl;
            }
            if (trojan_index / m_num_cols >= 1 &&
                trojan_index % m_num_cols <= m_num_cols - 2)
            {
                // int index = _router_id_to_index(
                //     trojan_router_id + m_num_cols + 1, m_num_cols,
                //     m_num_rows);
                int index = trojan_router_id + m_num_cols + 1;
                m_routers[index]->set_alert_flag(2);
                std::cout << "Router " << m_routers[index]->get_id()
                          << " is a propagation router\n"
                          << std::endl;
            }
            if (trojan_index / m_num_cols <= m_num_rows - 2 &&
                trojan_index % m_num_cols >= 1)
            {
                // int index = _router_id_to_index(
                //     trojan_router_id - m_num_cols - 1, m_num_cols,
                //     m_num_rows);
                int index = trojan_router_id - m_num_cols - 1;
                m_routers[index]->set_alert_flag(2);
                std::cout << "Router " << m_routers[index]->get_id()
                          << " is a propagation router\n"
                          << std::endl;
            }
            if (trojan_index / m_num_cols <= m_num_rows - 2 &&
                trojan_index % m_num_cols <= m_num_cols - 2)
            {
                // int index = _router_id_to_index(
                //     trojan_router_id - m_num_cols + 1, m_num_cols,
                //     m_num_rows);
                int index = trojan_router_id - m_num_cols + 1;
                m_routers[index]->set_alert_flag(2);
                std::cout << "Router " << m_routers[index]->get_id()
                          << " is a propagation router\n"
                          << std::endl;
            }
        }

        // set alert flags for routers connected to a trojan
        if (trojan_router_ids.size() > 0)
        {
            for (std::vector<int>::const_iterator i =
                     trojan_router_ids.begin();
                 i != trojan_router_ids.end(); ++i)
            {
                int trojan_router_id = *i;
                int trojan_index = _router_id_to_index(trojan_router_id,
                                                       m_num_cols, m_num_rows);
                if (trojan_index % m_num_cols != 0)
                {
                    // int index = _router_id_to_index(trojan_router_id - 1,
                    //                                 m_num_cols, m_num_rows);
                    int index = trojan_router_id - 1;
                    m_routers[index]->set_alert_flag(1);
                    m_routers[index]->set_alert_dirn("East");
                    std::cout
                        << "Router " << m_routers[index]->get_id()
                        << " is connected to a trojan in the East direction\n"
                        << std::endl;
                }
                if (trojan_index % m_num_cols != m_num_cols - 1)
                {
                    // int index = _router_id_to_index(trojan_router_id + 1,
                    //                                 m_num_cols, m_num_rows);
                    int index = trojan_router_id + 1;
                    m_routers[index]->set_alert_flag(1);
                    m_routers[index]->set_alert_dirn("West");
                    std::cout
                        << "Router " << m_routers[index]->get_id()
                        << " is connected to a trojan in the West direction\n"
                        << std::endl;
                }
                if (trojan_index / m_num_cols != m_num_cols - 1)
                {
                    // int index = _router_id_to_index(
                    //     trojan_router_id - m_num_cols, m_num_cols,
                    //     m_num_rows);
                    int index = trojan_router_id - m_num_cols;
                    m_routers[index]->set_alert_flag(1);
                    m_routers[index]->set_alert_dirn("North");
                    std::cout
                        << "Router " << m_routers[index]->get_id()
                        << " is connected to a trojan in the North direction\n"
                        << std::endl;
                }
                if (trojan_index / m_num_cols != 0)
                {
                    // int index = _router_id_to_index(
                    //     trojan_router_id + m_num_cols, m_num_cols,
                    //     m_num_rows);
                    int index = trojan_router_id + m_num_cols;
                    m_routers[index]->set_alert_flag(1);
                    m_routers[index]->set_alert_dirn("South");
                    std::cout
                        << "Router " << m_routers[index]->get_id()
                        << " is connected to a trojan in the South direction\n"
                        << std::endl;
                }
            }
        }
    }

    // record the network interfaces
    for (std::vector<ClockedObject *>::const_iterator i = p.netifs.begin();
         i != p.netifs.end(); ++i)
    {
        NetworkInterface *ni = safe_cast<NetworkInterface *>(*i);
        m_nis.push_back(ni);
        ni->init_net_ptr(this);
    }

    // Print Garnet version
    inform("Garnet version %s\n", garnetVersion);
}

void GarnetNetwork::init()
{
    Network::init();

    for (int i = 0; i < m_nodes; i++)
    {
        m_nis[i]->addNode(m_toNetQueues[i], m_fromNetQueues[i]);
    }

    // The topology pointer should have already been initialized in the
    // parent network constructor
    assert(m_topology_ptr != NULL);
    m_topology_ptr->createLinks(this);

    // Initialize topology specific parameters
    if (getNumRows() > 0)
    {
        // Only for Mesh topology
        // m_num_rows and m_num_cols are only used for
        // implementing XY or custom routing in RoutingUnit.cc
        m_num_rows = getNumRows();
        m_num_cols = m_routers.size() / m_num_rows;
        assert(m_num_rows * m_num_cols == m_routers.size());
    }
    else
    {
        m_num_rows = -1;
        m_num_cols = -1;
    }

    // FaultModel: declare each router to the fault model
    if (isFaultModelEnabled())
    {
        for (std::vector<Router *>::const_iterator i = m_routers.begin();
             i != m_routers.end(); ++i)
        {
            Router *router = safe_cast<Router *>(*i);
            [[maybe_unused]] int router_id = fault_model->declare_router(
                router->get_num_inports(), router->get_num_outports(),
                router->get_vc_per_vnet(), getBuffersPerDataVC(),
                getBuffersPerCtrlVC());
            assert(router_id == router->get_id());
            router->printAggregateFaultProbability(std::cout);
            router->printFaultVector(std::cout);
        }
    }
}

/*
 * This function creates a link from the Network Interface (NI)
 * into the Network.
 * It creates a Network Link from the NI to a Router and a Credit Link from
 * the Router to the NI
 */

void GarnetNetwork::makeExtInLink(NodeID global_src, SwitchID dest,
                                  BasicLink *link,
                                  std::vector<NetDest> &routing_table_entry)
{
    NodeID local_src = getLocalNodeID(global_src);
    assert(local_src < m_nodes);

    GarnetExtLink *garnet_link = safe_cast<GarnetExtLink *>(link);

    // GarnetExtLink is bi-directional
    NetworkLink *net_link = garnet_link->m_network_links[LinkDirection_In];
    net_link->setType(EXT_IN_);
    CreditLink *credit_link = garnet_link->m_credit_links[LinkDirection_In];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection dst_inport_dirn = "Local";

    m_max_vcs_per_vnet =
        std::max(m_max_vcs_per_vnet, m_routers[dest]->get_vc_per_vnet());

    /*
     * We check if a bridge was enabled at any end of the link.
     * The bridge is enabled if either of clock domain
     * crossing (CDC) or Serializer-Deserializer(SerDes) unit is
     * enabled for the link at each end. The bridge encapsulates
     * the functionality for both CDC and SerDes and is a Consumer
     * object similiar to a NetworkLink.
     *
     * If a bridge was enabled we connect the NI and Routers to
     * bridge before connecting the link. Example, if an external
     * bridge is enabled, we would connect:
     * NI--->NetworkBridge--->GarnetExtLink---->Router
     */
    if (garnet_link->extBridgeEn)
    {
        DPRINTF(RubyNetwork, "Enable external bridge for %s\n",
                garnet_link->name());
        m_nis[local_src]->addOutPort(
            garnet_link->extNetBridge[LinkDirection_In],
            garnet_link->extCredBridge[LinkDirection_In], dest,
            m_routers[dest]->get_vc_per_vnet());
    }
    else
    {
        m_nis[local_src]->addOutPort(net_link, credit_link, dest,
                                     m_routers[dest]->get_vc_per_vnet());
    }

    if (garnet_link->intBridgeEn)
    {
        DPRINTF(RubyNetwork, "Enable internal bridge for %s\n",
                garnet_link->name());
        m_routers[dest]->addInPort(
            dst_inport_dirn, garnet_link->intNetBridge[LinkDirection_In],
            garnet_link->intCredBridge[LinkDirection_In]);
    }
    else
    {
        m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    }
}

/*
 * This function creates a link from the Network to a NI.
 * It creates a Network Link from a Router to the NI and
 * a Credit Link from NI to the Router
 */

void GarnetNetwork::makeExtOutLink(SwitchID src, NodeID global_dest,
                                   BasicLink *link,
                                   std::vector<NetDest> &routing_table_entry)
{
    NodeID local_dest = getLocalNodeID(global_dest);
    assert(local_dest < m_nodes);
    assert(src < m_routers.size());
    assert(m_routers[src] != NULL);

    GarnetExtLink *garnet_link = safe_cast<GarnetExtLink *>(link);

    // GarnetExtLink is bi-directional
    NetworkLink *net_link = garnet_link->m_network_links[LinkDirection_Out];
    net_link->setType(EXT_OUT_);
    CreditLink *credit_link = garnet_link->m_credit_links[LinkDirection_Out];

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    PortDirection src_outport_dirn = "Local";

    m_max_vcs_per_vnet =
        std::max(m_max_vcs_per_vnet, m_routers[src]->get_vc_per_vnet());

    /*
     * We check if a bridge was enabled at any end of the link.
     * The bridge is enabled if either of clock domain
     * crossing (CDC) or Serializer-Deserializer(SerDes) unit is
     * enabled for the link at each end. The bridge encapsulates
     * the functionality for both CDC and SerDes and is a Consumer
     * object similiar to a NetworkLink.
     *
     * If a bridge was enabled we connect the NI and Routers to
     * bridge before connecting the link. Example, if an external
     * bridge is enabled, we would connect:
     * NI<---NetworkBridge<---GarnetExtLink<----Router
     */
    if (garnet_link->extBridgeEn)
    {
        DPRINTF(RubyNetwork, "Enable external bridge for %s\n",
                garnet_link->name());
        m_nis[local_dest]->addInPort(
            garnet_link->extNetBridge[LinkDirection_Out],
            garnet_link->extCredBridge[LinkDirection_Out]);
    }
    else
    {
        m_nis[local_dest]->addInPort(net_link, credit_link);
    }

    if (garnet_link->intBridgeEn)
    {
        DPRINTF(RubyNetwork, "Enable internal bridge for %s\n",
                garnet_link->name());
        m_routers[src]->addOutPort(
            src_outport_dirn, garnet_link->intNetBridge[LinkDirection_Out],
            routing_table_entry, link->m_weight,
            garnet_link->intCredBridge[LinkDirection_Out],
            m_routers[src]->get_vc_per_vnet());
    }
    else
    {
        m_routers[src]->addOutPort(
            src_outport_dirn, net_link, routing_table_entry, link->m_weight,
            credit_link, m_routers[src]->get_vc_per_vnet());
    }
}

/*
 * This function creates an internal network link between two routers.
 * It adds both the network link and an opposite credit link.
 */

void GarnetNetwork::makeInternalLink(SwitchID src, SwitchID dest,
                                     BasicLink *link,
                                     std::vector<NetDest> &routing_table_entry,
                                     PortDirection src_outport_dirn,
                                     PortDirection dst_inport_dirn)
{
    GarnetIntLink *garnet_link = safe_cast<GarnetIntLink *>(link);

    // GarnetIntLink is unidirectional
    NetworkLink *net_link = garnet_link->m_network_link;
    net_link->setType(INT_);
    CreditLink *credit_link = garnet_link->m_credit_link;

    m_networklinks.push_back(net_link);
    m_creditlinks.push_back(credit_link);

    m_max_vcs_per_vnet = std::max(m_max_vcs_per_vnet,
                                  std::max(m_routers[dest]->get_vc_per_vnet(),
                                           m_routers[src]->get_vc_per_vnet()));

    /*
     * We check if a bridge was enabled at any end of the link.
     * The bridge is enabled if either of clock domain
     * crossing (CDC) or Serializer-Deserializer(SerDes) unit is
     * enabled for the link at each end. The bridge encapsulates
     * the functionality for both CDC and SerDes and is a Consumer
     * object similiar to a NetworkLink.
     *
     * If a bridge was enabled we connect the NI and Routers to
     * bridge before connecting the link. Example, if a source
     * bridge is enabled, we would connect:
     * Router--->NetworkBridge--->GarnetIntLink---->Router
     */
    if (garnet_link->dstBridgeEn)
    {
        DPRINTF(RubyNetwork, "Enable destination bridge for %s\n",
                garnet_link->name());
        m_routers[dest]->addInPort(dst_inport_dirn, garnet_link->dstNetBridge,
                                   garnet_link->dstCredBridge);
    }
    else
    {
        m_routers[dest]->addInPort(dst_inport_dirn, net_link, credit_link);
    }

    if (garnet_link->srcBridgeEn)
    {
        DPRINTF(RubyNetwork, "Enable source bridge for %s\n",
                garnet_link->name());
        m_routers[src]->addOutPort(src_outport_dirn, garnet_link->srcNetBridge,
                                   routing_table_entry, link->m_weight,
                                   garnet_link->srcCredBridge,
                                   m_routers[dest]->get_vc_per_vnet());
    }
    else
    {
        m_routers[src]->addOutPort(
            src_outport_dirn, net_link, routing_table_entry, link->m_weight,
            credit_link, m_routers[dest]->get_vc_per_vnet());
    }
}

// Total routers in the network
int GarnetNetwork::getNumRouters()
{
    return m_routers.size();
}

// Get ID of router connected to a NI.
int GarnetNetwork::get_router_id(int global_ni, int vnet)
{
    NodeID local_ni = getLocalNodeID(global_ni);

    return m_nis[local_ni]->get_router_id(vnet);
}

void GarnetNetwork::regStats()
{
    Network::regStats();

    // Packets
    m_packets_received.init(m_virtual_networks)
        .name(name() + ".packets_received")
        .flags(statistics::pdf | statistics::total | statistics::nozero |
               statistics::oneline);

    m_packets_injected.init(m_virtual_networks)
        .name(name() + ".packets_injected")
        .flags(statistics::pdf | statistics::total | statistics::nozero |
               statistics::oneline);

    m_packet_network_latency.init(m_virtual_networks)
        .name(name() + ".packet_network_latency")
        .flags(statistics::oneline);

    m_packet_queueing_latency.init(m_virtual_networks)
        .name(name() + ".packet_queueing_latency")
        .flags(statistics::oneline);

    for (int i = 0; i < m_virtual_networks; i++)
    {
        m_packets_received.subname(i, csprintf("vnet-%i", i));
        m_packets_injected.subname(i, csprintf("vnet-%i", i));
        m_packet_network_latency.subname(i, csprintf("vnet-%i", i));
        m_packet_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_packet_vnet_latency.name(name() + ".average_packet_vnet_latency")
        .flags(statistics::oneline);
    m_avg_packet_vnet_latency = m_packet_network_latency / m_packets_received;

    m_avg_packet_vqueue_latency.name(name() + ".average_packet_vqueue_latency")
        .flags(statistics::oneline);
    m_avg_packet_vqueue_latency =
        m_packet_queueing_latency / m_packets_received;

    m_avg_packet_network_latency.name(name() +
                                      ".average_packet_network_latency");
    m_avg_packet_network_latency =
        sum(m_packet_network_latency) / sum(m_packets_received);

    m_avg_packet_queueing_latency.name(name() +
                                       ".average_packet_queueing_latency");
    m_avg_packet_queueing_latency =
        sum(m_packet_queueing_latency) / sum(m_packets_received);

    m_avg_packet_latency.name(name() + ".average_packet_latency");
    m_avg_packet_latency =
        m_avg_packet_network_latency + m_avg_packet_queueing_latency;

    // Flits
    m_flits_received.init(m_virtual_networks)
        .name(name() + ".flits_received")
        .flags(statistics::pdf | statistics::total | statistics::nozero |
               statistics::oneline);

    m_flits_injected.init(m_virtual_networks)
        .name(name() + ".flits_injected")
        .flags(statistics::pdf | statistics::total | statistics::nozero |
               statistics::oneline);

    m_flit_network_latency.init(m_virtual_networks)
        .name(name() + ".flit_network_latency")
        .flags(statistics::oneline);

    m_flit_queueing_latency.init(m_virtual_networks)
        .name(name() + ".flit_queueing_latency")
        .flags(statistics::oneline);

    for (int i = 0; i < m_virtual_networks; i++)
    {
        m_flits_received.subname(i, csprintf("vnet-%i", i));
        m_flits_injected.subname(i, csprintf("vnet-%i", i));
        m_flit_network_latency.subname(i, csprintf("vnet-%i", i));
        m_flit_queueing_latency.subname(i, csprintf("vnet-%i", i));
    }

    m_avg_flit_vnet_latency.name(name() + ".average_flit_vnet_latency")
        .flags(statistics::oneline);
    m_avg_flit_vnet_latency = m_flit_network_latency / m_flits_received;

    m_avg_flit_vqueue_latency.name(name() + ".average_flit_vqueue_latency")
        .flags(statistics::oneline);
    m_avg_flit_vqueue_latency = m_flit_queueing_latency / m_flits_received;

    m_avg_flit_network_latency.name(name() + ".average_flit_network_latency");
    m_avg_flit_network_latency =
        sum(m_flit_network_latency) / sum(m_flits_received);

    m_avg_flit_queueing_latency.name(name() +
                                     ".average_flit_queueing_latency");
    m_avg_flit_queueing_latency =
        sum(m_flit_queueing_latency) / sum(m_flits_received);

    m_avg_flit_latency.name(name() + ".average_flit_latency");
    m_avg_flit_latency =
        m_avg_flit_network_latency + m_avg_flit_queueing_latency;

    // Hops
    m_avg_hops.name(name() + ".average_hops");
    m_avg_hops = m_total_hops / sum(m_flits_received);

    // Links
    m_total_ext_in_link_utilization.name(name() + ".ext_in_link_utilization");
    m_total_ext_out_link_utilization.name(name() +
                                          ".ext_out_link_utilization");
    m_total_int_link_utilization.name(name() + ".int_link_utilization");
    m_average_link_utilization.name(name() + ".avg_link_utilization");
    m_average_vc_load.init(m_virtual_networks * m_max_vcs_per_vnet)
        .name(name() + ".avg_vc_load")
        .flags(statistics::pdf | statistics::total | statistics::nozero |
               statistics::oneline);

    // Traffic distribution
    for (int source = 0; source < m_routers.size(); ++source)
    {
        m_data_traffic_distribution.push_back(
            std::vector<statistics::Scalar *>());
        m_ctrl_traffic_distribution.push_back(
            std::vector<statistics::Scalar *>());

        for (int dest = 0; dest < m_routers.size(); ++dest)
        {
            statistics::Scalar *data_packets = new statistics::Scalar();
            statistics::Scalar *ctrl_packets = new statistics::Scalar();

            data_packets->name(name() + ".data_traffic_distribution." + "n" +
                               std::to_string(source) + "." + "n" +
                               std::to_string(dest));
            m_data_traffic_distribution[source].push_back(data_packets);

            ctrl_packets->name(name() + ".ctrl_traffic_distribution." + "n" +
                               std::to_string(source) + "." + "n" +
                               std::to_string(dest));
            m_ctrl_traffic_distribution[source].push_back(ctrl_packets);
        }
    }
}

void GarnetNetwork::collateStats()
{
    RubySystem *rs = params().ruby_system;
    double time_delta = double(curCycle() - rs->getStartCycle());

    for (int i = 0; i < m_networklinks.size(); i++)
    {
        link_type type = m_networklinks[i]->getType();
        int activity = m_networklinks[i]->getLinkUtilization();

        if (type == EXT_IN_)
            m_total_ext_in_link_utilization += activity;
        else if (type == EXT_OUT_)
            m_total_ext_out_link_utilization += activity;
        else if (type == INT_)
            m_total_int_link_utilization += activity;

        m_average_link_utilization += (double(activity) / time_delta);

        std::vector<unsigned int> vc_load = m_networklinks[i]->getVcLoad();
        for (int j = 0; j < vc_load.size(); j++)
        {
            m_average_vc_load[j] += ((double)vc_load[j] / time_delta);
        }
    }

    // Ask the routers to collate their statistics
    for (int i = 0; i < m_routers.size(); i++)
    {
        m_routers[i]->collateStats();
    }
}

void GarnetNetwork::resetStats()
{
    for (int i = 0; i < m_routers.size(); i++)
    {
        m_routers[i]->resetStats();
    }
    for (int i = 0; i < m_networklinks.size(); i++)
    {
        m_networklinks[i]->resetStats();
    }
    for (int i = 0; i < m_creditlinks.size(); i++)
    {
        m_creditlinks[i]->resetStats();
    }
}

void GarnetNetwork::print(std::ostream &out) const
{
    out << "[GarnetNetwork]";
}

void GarnetNetwork::update_traffic_distribution(RouteInfo route)
{
    int src_node = route.src_router;
    int dest_node = route.dest_router;
    int vnet = route.vnet;

    if (m_vnet_type[vnet] == DATA_VNET_)
        (*m_data_traffic_distribution[src_node][dest_node])++;
    else
        (*m_ctrl_traffic_distribution[src_node][dest_node])++;
}

uint32_t GarnetNetwork::functionalWrite(Packet *pkt)
{
    uint32_t num_functional_writes = 0;

    for (unsigned int i = 0; i < m_routers.size(); i++)
    {
        num_functional_writes += m_routers[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_nis.size(); ++i)
    {
        num_functional_writes += m_nis[i]->functionalWrite(pkt);
    }

    for (unsigned int i = 0; i < m_networklinks.size(); ++i)
    {
        num_functional_writes += m_networklinks[i]->functionalWrite(pkt);
    }

    return num_functional_writes;
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
