###############################################################################
# Copyright Centaurus Computing Plc - 2016
#
#This file is part of riosocket-basic.
#riosocket-basic is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.
#
#riosocket-basic is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License
#along with riosocket-basic.  If not, see <http://www.gnu.org/licenses/>.
#
################################################################################
#!/bin/bash

CORES=$((`cat /proc/cpuinfo | grep processor | tail -1 | awk '{print $3}'`+1))
PROC=$(($CORES/4))

db=$((`cat /proc/interrupts | grep "tsi721-idb" | cut -f 1 -d ":"`))
dmad=$((`cat /proc/interrupts | grep "tsi721-dmad" | cut -f 1 -d ":"`))
dmai=$((`cat /proc/interrupts | grep "tsi721-dmai" | cut -f 1 -d ":"`))

/etc/init.d/irqbalance stop

echo 01 > /proc/irq/$db/smp_affinity

if (( $PROC > 1 ))
then
        echo 10 >  /proc/irq/$dmad/smp_affinity
        echo 20 >  /proc/irq/$dmai/smp_affinity
else
        echo 02 >  /proc/irq/$dmad/smp_affinity
        echo 04 >  /proc/irq/$dmai/smp_affinity
fi


echo 32768 > /proc/sys/net/core/rps_sock_flow_entries

sysctl -w net.ipv4.tcp_timestamps=0
sysctl -w net.ipv4.tcp_sack=0
sysctl -w net.core.netdev_max_backlog=250000

