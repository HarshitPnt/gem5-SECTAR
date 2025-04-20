/*
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

#include "mem/ruby/network/garnet/RoutingUnit.hh"

#include "base/cast.hh"
#include "base/compiler.hh"
#include "debug/RubyNetwork.hh"
#include "mem/ruby/network/garnet/InputUnit.hh"
#include "mem/ruby/network/garnet/Router.hh"
#include "mem/ruby/slicc_interface/Message.hh"

#define HT_PROBABILITY 10

namespace gem5
{

namespace ruby
{

namespace garnet
{

RoutingUnit::RoutingUnit(Router *router)
{
    m_router = router;
    m_routing_table.clear();
    m_weight_table.clear();
}

void RoutingUnit::addRoute(std::vector<NetDest> &routing_table_entry)
{
    if (routing_table_entry.size() > m_routing_table.size())
    {
        m_routing_table.resize(routing_table_entry.size());
    }
    for (int v = 0; v < routing_table_entry.size(); v++)
    {
        m_routing_table[v].push_back(routing_table_entry[v]);
    }
}

void RoutingUnit::addWeight(int link_weight)
{
    m_weight_table.push_back(link_weight);
}

bool RoutingUnit::supportsVnet(int vnet, std::vector<int> sVnets)
{
    // If all vnets are supported, return true
    if (sVnets.size() == 0)
    {
        return true;
    }

    // Find the vnet in the vector, return true
    if (std::find(sVnets.begin(), sVnets.end(), vnet) != sVnets.end())
    {
        return true;
    }

    // Not supported vnet
    return false;
}

/*
 * This is the default routing algorithm in garnet.
 * The routing table is populated during topology creation.
 * Routes can be biased via weight assignments in the topology file.
 * Correct weight assignments are critical to provide deadlock avoidance.
 */
int RoutingUnit::lookupRoutingTable(int vnet, NetDest msg_destination)
{
    // First find all possible output link candidates
    // For ordered vnet, just choose the first
    // (to make sure different packets don't choose different routes)
    // For unordered vnet, randomly choose any of the links
    // To have a strict ordering between links, they should be given
    // different weights in the topology file

    int output_link = -1;
    int min_weight = INFINITE_;
    std::vector<int> output_link_candidates;
    int num_candidates = 0;

    // Identify the minimum weight among the candidate output links
    for (int link = 0; link < m_routing_table[vnet].size(); link++)
    {
        if (msg_destination.intersectionIsNotEmpty(
                m_routing_table[vnet][link]))
        {

            if (m_weight_table[link] <= min_weight)
                min_weight = m_weight_table[link];
        }
    }

    // Collect all candidate output links with this minimum weight
    for (int link = 0; link < m_routing_table[vnet].size(); link++)
    {
        if (msg_destination.intersectionIsNotEmpty(
                m_routing_table[vnet][link]))
        {

            if (m_weight_table[link] == min_weight)
            {
                num_candidates++;
                output_link_candidates.push_back(link);
            }
        }
    }

    if (output_link_candidates.size() == 0)
    {
        fatal("Fatal Error:: No Route exists from this Router.");
        exit(0);
    }

    // Randomly select any candidate output link
    int candidate = 0;
    if (!(m_router->get_net_ptr())->isVNetOrdered(vnet))
        candidate = rand() % num_candidates;

    output_link = output_link_candidates.at(candidate);
    return output_link;
}

void RoutingUnit::addInDirection(PortDirection inport_dirn, int inport_idx)
{
    m_inports_dirn2idx[inport_dirn] = inport_idx;
    m_inports_idx2dirn[inport_idx] = inport_dirn;
}

void RoutingUnit::addOutDirection(PortDirection outport_dirn, int outport_idx)
{
    m_outports_dirn2idx[outport_dirn] = outport_idx;
    m_outports_idx2dirn[outport_idx] = outport_dirn;
}

// outportCompute() is called by the InputUnit
// It calls the routing table by default.
// A template for adaptive topology-specific routing algorithm
// implementations using port directions rather than a static routing
// table is provided here.

bool RoutingUnit::isFlitDeflected(RouteInfo route, int inport,
                                  PortDirection inport_dirn)
{
    [[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_diff = dest_x - my_x;
    int y_diff = dest_y - my_y;
    if ((x_diff != 0 && (inport_dirn == "North" || inport_dirn == "South")) ||
        (x_diff > 0 && inport_dirn == "East") ||
        (x_diff < 0 && inport_dirn == "West") ||
        (x_diff == 0 && y_diff < 0 && inport_dirn == "North") ||
        (x_diff == 0 && y_diff > 0 && inport_dirn == "South"))
        return true;
    return false;
}

int RoutingUnit::outportCompute(RouteInfo route, int inport,
                                PortDirection inport_dirn, int id)
{
    int outport = -1;

    if (route.dest_router == m_router->get_id())
    {

        // Multiple NIs may be connected to this router,
        // all with output port direction = "Local"
        // Get exact outport id from table
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        std::cout << "Ejecting to local outport at router "
                  << m_router->get_id() << std::endl;
        return outport;
    }

    // Routing Algorithm set in GarnetNetwork.py
    // Can be over-ridden from command line using --routing-algorithm = 1
    RoutingAlgorithm routing_algorithm =
        (RoutingAlgorithm)m_router->get_net_ptr()->getRoutingAlgorithm();

    switch (routing_algorithm)
    {
    case TABLE_:
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        break;
    case XY_:
        outport = outportComputeXY(route, inport, inport_dirn);
        break;
    // any custom algorithm
    case CUSTOM_:
        outport = outportComputeCustom(route, inport, inport_dirn);
        break;
    case SECTAR_:
        // TODO: Check for deflected packet
        // bool is_deflected =
        //     isFlitDeflected(route, inport, inport_dirn);
        // if (is_deflected)
        // {
        //     // do something
        //     ;
        // }
        outport = outportComputeSectar(route, inport, inport_dirn, id);
        break;
    default:
    case HT_:
        outport = outportComputeHT(route, inport, inport_dirn);
        break;
        outport = lookupRoutingTable(route.vnet, route.net_dest);
        break;
    }

    assert(outport != -1);
    return outport;
}

// XY routing implemented using port directions
// Only for reference purpose in a Mesh
// By default Garnet uses the routing table
int RoutingUnit::outportComputeXY(RouteInfo route, int inport,
                                  PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";

    [[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > 0)
    {
        if (x_dirn)
        {
            assert(inport_dirn == "Local" || inport_dirn == "West");
            outport_dirn = "East";
        }
        else
        {
            assert(inport_dirn == "Local" || inport_dirn == "East");
            outport_dirn = "West";
        }
    }
    else if (y_hops > 0)
    {
        if (y_dirn)
        {
            // "Local" or "South" or "West" or "East"
            assert(inport_dirn != "North");
            outport_dirn = "North";
        }
        else
        {
            // "Local" or "North" or "West" or "East"
            assert(inport_dirn != "South");
            outport_dirn = "South";
        }
    }
    else
    {
        // x_hops == 0 and y_hops == 0
        // this is not possible
        // already checked that in outportCompute() function
        panic("x_hops == y_hops == 0");
    }
    std::cout << "Router id: " << m_router->get_id() << " Current x: " << my_x
              << " Destination x: " << dest_x << " Current y: " << my_y
              << " Destination y: " << dest_y << " inport direction "
              << inport_dirn << std::endl;
    return m_outports_dirn2idx[outport_dirn];
}

// Template for implementing custom routing algorithm
// using port directions. (Example adaptive)
int RoutingUnit::outportComputeCustom(RouteInfo route, int inport,
                                      PortDirection inport_dirn)
{
    panic("%s placeholder executed", __FUNCTION__);
}

bool _connected_to_trojan(std::vector<PortDirection> alert_dirns,
                          PortDirection alert_dirn)
{
    if (std::find(alert_dirns.begin(), alert_dirns.end(), alert_dirn) !=
        alert_dirns.end())
        return true;
    return false;
}

PortDirection _random_non_trojan_dirn(std::vector<PortDirection> alert_dirns,
                                      int row, int rows, int col, int cols)
{
    std::vector<PortDirection> directions = {"North", "South", "East", "West"};
    std::vector<PortDirection> non_trojan_dirns;
    for (auto dirn : directions)
    {
        if (std::find(alert_dirns.begin(), alert_dirns.end(), dirn) ==
            alert_dirns.end())
        {
            if (row == rows - 1 && dirn == "North")
                continue;
            if (row == 0 && dirn == "South")
                continue;
            if (col == 0 && dirn == "West")
                continue;
            if (col == cols - 1 && dirn == "East")
                continue;
            non_trojan_dirns.push_back(dirn);
        }
    }
    for (auto dirn : non_trojan_dirns)
    {
        std::cout << dirn << " ";
    }
    std::cout << std::endl;
    if (non_trojan_dirns.size() == 0)
        return "Local";
    int random_index = rand() % non_trojan_dirns.size();
    return non_trojan_dirns[random_index];
}

PortDirection _opposite_dirn(PortDirection dirn)
{
    if (dirn == "North")
        return "South";
    else if (dirn == "South")
        return "North";
    else if (dirn == "East")
        return "West";
    else if (dirn == "West")
        return "East";
    else
        return "Local";
}

// Routing algorithm for SECTAR
int RoutingUnit::outportComputeSectar(RouteInfo route, int inport,
                                      PortDirection inport_dirn, int id)
{
    // panic("%s placeholder executed", __FUNCTION__);
    PortDirection outport_dirn = "Unknown";
    [[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_diff = dest_x - my_x;
    int y_diff = dest_y - my_y;
    PortDirection outport_dirn_trojan = "Unknown";
    uint8_t alert_flag = m_router->get_alert_flag();
    int routing_flag = 0;
    std::vector<PortDirection> available_directions;
    bool trojan_dest = false;
    if (m_router->is_trojan())
    {
        // randomly get any output port direction
        std::vector<PortDirection> directions = {"North", "South", "East",
                                                 "West"};
        // get only directions that are spatially valid
        for (auto dir : directions)
        {
            if (dir == "North" && my_y == num_rows - 1)
                continue;
            if (dir == "South" && my_y == 0)
                continue;
            if (dir == "West" && my_x == 0)
                continue;
            if (dir == "East" && my_x == num_cols - 1)
                continue;
            available_directions.push_back(dir);
        }
        // choose a direction randomly
        int random_index = rand() % available_directions.size();
        outport_dirn_trojan = available_directions[random_index];
    }
    if (alert_flag == 1 && !m_router->is_trojan())
    {
        // std::cout << "Router " << m_router->get_id()
        //           << " is connected to a trojan at " << num_cols * 3 + 3
        //           << std::endl;
        // directly connected to a trojan
        std::vector<PortDirection> alert_dirn = m_router->get_alert_dirn();
        // std::cout << "Alert direction: " << alert_dirn << " Inport
        // direction
        // "
        //           << inport_dirn << std::endl;
        if (x_diff != 0)
        {
            if (x_diff > 0)
            {
                if (_connected_to_trojan(alert_dirn, "East"))
                {
                    if (dest_x - 1 == my_x && dest_y == my_y)
                    {
                        outport_dirn = "East";
                        trojan_dest = true;
                    }
                    else if (y_diff <= 0)
                    {
                        outport_dirn = "South";
                        outport_dirn =
                            y_diff == 0 && inport_dirn == outport_dirn
                                ? "North"
                                : outport_dirn;
                    }
                    else
                    {
                        outport_dirn = "North";
                    }
                    if (!trojan_dest)
                    {

                        outport_dirn =
                            std::find(alert_dirn.begin(), alert_dirn.end(),
                                      outport_dirn) != alert_dirn.end()
                                ? inport_dirn
                                : outport_dirn;
                        outport_dirn = outport_dirn == "Local"
                                           ? _random_non_trojan_dirn(
                                                 alert_dirn, my_y, num_rows,
                                                 my_x, num_cols)
                                           : outport_dirn;
                    }
                }
                else
                    outport_dirn = "East";
            }
            else
            {
                if (_connected_to_trojan(alert_dirn, "West"))
                {
                    if (dest_y == my_y && dest_x + 1 == my_x)
                    {
                        outport_dirn = "West";
                        trojan_dest = true;
                    }
                    else if (y_diff <= 0)
                    {
                        outport_dirn = "South";
                        outport_dirn =
                            (y_diff == 0 && inport_dirn == outport_dirn) ||
                                    (my_y == 0)
                                ? "North"
                                : outport_dirn;
                    }
                    else
                    {
                        outport_dirn = "North";
                    }
                    if (!trojan_dest)
                    {

                        outport_dirn =
                            std::find(alert_dirn.begin(), alert_dirn.end(),
                                      outport_dirn) != alert_dirn.end()
                                ? inport_dirn
                                : outport_dirn;
                        outport_dirn = outport_dirn == "Local"
                                           ? _random_non_trojan_dirn(
                                                 alert_dirn, my_y, num_rows,
                                                 my_x, num_cols)
                                           : outport_dirn;
                    }
                }
                else
                    outport_dirn = "West";
            }
        }
        else if (y_diff != 0)
        {
            if (y_diff > 0)
            {
                if (_connected_to_trojan(alert_dirn, "North"))
                {
                    if (dest_x == my_x && dest_y - 1 == my_y)
                    {
                        outport_dirn = "North";
                        trojan_dest = true;
                    }
                    else if (x_diff <= 0)
                    {
                        outport_dirn = "West";
                        outport_dirn =
                            x_diff == 0 && inport_dirn == outport_dirn
                                ? "East"
                                : outport_dirn;
                    }
                    else
                    {
                        outport_dirn = "East";
                    }
                    if (!trojan_dest)
                    {

                        outport_dirn =
                            std::find(alert_dirn.begin(), alert_dirn.end(),
                                      outport_dirn) != alert_dirn.end()
                                ? inport_dirn
                                : outport_dirn;
                        outport_dirn = outport_dirn == "Local"
                                           ? _random_non_trojan_dirn(
                                                 alert_dirn, my_y, num_rows,
                                                 my_x, num_cols)
                                           : outport_dirn;
                    }
                }
                else
                    outport_dirn = "North";
            }
            else
            {
                if (_connected_to_trojan(alert_dirn, "South"))
                {
                    if (dest_x == my_x && dest_y + 1 == my_y)
                    {
                        outport_dirn = "South";
                        trojan_dest = true;
                    }
                    else if (x_diff <= 0)
                    {
                        outport_dirn = "West";
                        outport_dirn =
                            (x_diff == 0 && inport_dirn == outport_dirn) ||
                                    (my_x == 0)
                                ? "East"
                                : outport_dirn;
                    }
                    else
                    {
                        outport_dirn = "East";
                    }
                    if (!trojan_dest)
                    {

                        outport_dirn =
                            std::find(alert_dirn.begin(), alert_dirn.end(),
                                      outport_dirn) != alert_dirn.end()
                                ? inport_dirn
                                : outport_dirn;
                        outport_dirn = outport_dirn == "Local"
                                           ? _random_non_trojan_dirn(
                                                 alert_dirn, my_y, num_rows,
                                                 my_x, num_cols)
                                           : outport_dirn;
                    }
                }
                else
                    outport_dirn = "South";
            }
        }
        else
        {
            panic("x_hops == y_hops == 0");
        }
        routing_flag = 1;
    }
    else if (alert_flag == 2 && !m_router->is_trojan())
    {
        // indirectly connected to a trojan
        // or no trojan
        // std::cout << "Router " << m_router->get_id()
        //           << " is a propagation router\n"
        //   << std::endl;
        // std::cout << "Alert direction: " << m_router->get_alert_dirn()
        // << std::endl;
        if ((x_diff < 0 && (inport_dirn == "West")) ||
            (x_diff > 0 && (inport_dirn == "East")))
        {
            if (y_diff < 0)
                outport_dirn = "South";
            else
                outport_dirn = "North";
        }
        else if (x_diff < 0 && (inport_dirn != "West"))
            outport_dirn = "West";
        else if (x_diff > 0 && (inport_dirn != "East"))
            outport_dirn = "East";
        else if (y_diff < 0)
            outport_dirn = "South";
        else if (y_diff > 0)
            outport_dirn = "North";
        else
        {
            // std::cout << x_diff << y_diff << std::endl;
            panic("x_hops == y_hops == 0");
        }
        // else if (x_diff < 0 && (inport_dirn == "South" || inport_dirn ==
        // "Local" || inport_dirn == "East"))
        //     outport_dirn = "West";
        // else if (x_diff > 0 && (inport_dirn == "North" || inport_dirn ==
        // "Local" || inport_dirn == "West"))
        //     outport_dirn = "East";
        routing_flag = 1;
    }
    else if (alert_flag == 0 || m_router->is_trojan())
    {
        int x_hops = abs(dest_x - my_x);
        int y_hops = abs(dest_y - my_y);

        bool x_dirn = (dest_x >= my_x);
        bool y_dirn = (dest_y >= my_y);
        if (x_hops > 0)
        {
            if (x_dirn)
            {
                assert(inport_dirn == "Local" || inport_dirn == "West");
                outport_dirn = "East";
            }
            else
            {
                assert(inport_dirn == "Local" || inport_dirn == "East");
                outport_dirn = "West";
            }
        }
        else if (y_hops > 0)
        {
            if (y_dirn)
            {
                // "Local" or "South" or "West" or "East"
                assert(inport_dirn != "North");
                outport_dirn = "North";
            }
            else
            {
                // "Local" or "North" or "West" or "East"
                assert(inport_dirn != "South");
                outport_dirn = "South";
            }
        }
        else
        {
            // x_hops == 0 and y_hops == 0
            // this is not possible
            // already checked that in outportCompute() function
            panic("x_hops == y_hops == 0");
        }
    }

    // std::cout << "Outport direction: " << outport_dirn << std::endl;
    if (outport_dirn_trojan != "Unknown")
    {
        // with 10% probability choose the trojan direction
        // else choose the normal direction
        int random_index = rand() % HT_PROBABILITY;
        if (random_index == 0)
        {
            outport_dirn = outport_dirn_trojan;
            routing_flag = 2;
        }
    }
    if (routing_flag == 0)
    {
        // std::cout << "XY Routing " << "Received packet at router "
        //           << m_router->get_id() << " with flit id " << id
        //           << " with dest id " << dest_id << " and inport direction "
        //           << inport_dirn << " sending to router " << outport_dirn
        //           << std::endl;
        ;
    }
    else if (routing_flag == 1)
    {
        // std::cout << "SECTAR Routing " << "Received packet at router "
        //           << m_router->get_id() << " with flit id " << id
        //           << " with dest id " << dest_id << " and inport direction "
        //           << inport_dirn << " sending to router " << outport_dirn
        //           << std::endl;
        ;
    }
    else if (routing_flag == 2)
    {
        // std::cout << "Trojan Routing " << "Received packet at router "
        //           << m_router->get_id() << " with flit id " << id
        //           << " with dest id " << dest_id << " and inport direction "
        //           << inport_dirn << " sending to router " << outport_dirn
        //           << std::endl;
        // for (auto dirn : available_directions)
        // {
        //     std::cout << dirn << " ";
        // }
        // std::cout << std::endl;
        ;
    }
    if (inport_dirn == "Local")
    {
        std::cout << "Destination router " << dest_id << std::endl;
    }

    return m_outports_dirn2idx[outport_dirn];
}

int RoutingUnit::outportComputeHT(RouteInfo route, int inport,
                                  PortDirection inport_dirn)
{
    PortDirection outport_dirn = "Unknown";
    PortDirection outport_dirn_trojan = "Unknown";

    [[maybe_unused]] int num_rows = m_router->get_net_ptr()->getNumRows();
    int num_cols = m_router->get_net_ptr()->getNumCols();
    assert(num_rows > 0 && num_cols > 0);

    int my_id = m_router->get_id();
    int my_x = my_id % num_cols;
    int my_y = my_id / num_cols;

    int dest_id = route.dest_router;
    int dest_x = dest_id % num_cols;
    int dest_y = dest_id / num_cols;

    int x_hops = abs(dest_x - my_x);
    int y_hops = abs(dest_y - my_y);

    bool x_dirn = (dest_x >= my_x);
    bool y_dirn = (dest_y >= my_y);

    // already checked that in outportCompute() function
    assert(!(x_hops == 0 && y_hops == 0));

    if (x_hops > 0)
    {
        if (x_dirn)
        {
            outport_dirn = "East";
        }
        else
        {
            outport_dirn = "West";
        }
    }
    else if (y_hops > 0)
    {
        if (y_dirn)
        {
            // "Local" or "South" or "West" or "East"
            outport_dirn = "North";
        }
        else
        {
            // "Local" or "North" or "West" or "East"
            outport_dirn = "South";
        }
    }
    else
    {
        // x_hops == 0 and y_hops == 0
        // this is not possible
        // already checked that in outportCompute() function
        panic("x_hops == y_hops == 0");
    }
    std::vector<PortDirection> directions = {"North", "South", "East", "West"};
    std::vector<PortDirection> available_directions;
    // get only directions that are spatially valid
    for (auto dir : directions)
    {
        if (dir == "North" && my_y == num_rows - 1)
            continue;
        if (dir == "South" && my_y == 0)
            continue;
        if (dir == "West" && my_x == 0)
            continue;
        if (dir == "East" && my_x == num_cols - 1)
            continue;
        available_directions.push_back(dir);
    }
    // choose a direction randomly
    int random_index = rand() % available_directions.size();
    outport_dirn_trojan = available_directions[random_index];
    if (outport_dirn_trojan != "Unknown")
    {
        // with 10% probability choose the trojan direction
        // else choose the normal direction
        int random_index = rand() % HT_PROBABILITY;
        if (random_index == 0)
        {
            outport_dirn = outport_dirn_trojan;
        }
    }

    return m_outports_dirn2idx[outport_dirn];
}

} // namespace garnet
} // namespace ruby
} // namespace gem5
