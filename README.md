# riosocket
Centaurus Computing's RIOSockets package implements a socket interface over RapidIO® fabric enabling unmodified socket application to run on top of RapidIO® fabric . This  enables the usage of ubiquitous software ecosystem developed around Ethernet on RapidIO® fabric, thus 
reducing the time to market and total cost of ownership for the user.

RIOSockets  package enables very high application level data stream performance in excess of 14 Gbit/s using a Gen2(5 Gbps) x4 RapidIO® links with low CPU utilization, thus freeing up the CPU to consume the high speed data being deposited over the RapidIO® link.

RIOSockets package provides two socket message data paths. A message can either go through the standard TCP/IP stack or bypass it depending upon the application being used. By providing the ability to bypass the TCP/IP stack, very low small packet jitterless latencies can be achieved making the solution ideal for latency sensitive applications like high frequency trading. By implementing a socket switch in a single socket library, an user application can decide at run-time the data path to be selected.

The RIOSocket package also provides  a fail-safe mechanism wherein when the RapidIO® link fails then the Ethernet link is used as a fall back option enabling high application link reliability. 

Lastly, RIOSocket package supports multiple NICs thus enabling features like link-aggregation and redundancy making this package ideal for high performance fault-tolerant system. 

With ability to deploy the above features on a cluster comprising of 100s of nodes and ability to run unmodified versions of open standard middleware like Hadoop and OpenMPI, large HPC clusters can be built using RapidIO® fabric. 

#Package Information
In view of user flexibility, RIOSocket package is functionally split into two sub-packages, namely RIOSocket – Basic and RIOSocket – Advanced. The following table shows the functionality provided in each of these packages.

#RIOSocket – Basic Features

1. Ethernet emulation over RapidIO fabric using standard TCP/IP stack
2. Ability to run standard unmodified Socket library based utilities like sftp, scp, etc on RapidIO fabric 
3. Ability to run standard communication middlewares like Hadoop or OpenMPI
4. Sustained application level throughput measured by standard iperf utility in excess of 14 Gbps.
5. Support for  100s of node support
6. Support for x86 architecture. Support on other architectures can be added on request basis.

#RIOSocket – Advanced Features

1. RapidIO fabric enumeration
2. Ethernet emulation over RapidIO fabric using standard TCP/IP stack
3. TCP/IP Stack bypass for latency sensitive application achieving sub-two micro second latency. 
4. Socket Switch to select between the above two transport paths
5. Multiple NIC support providing link aggregation and redundancies
6. Ethernet fail-safe on RapidIO link failure
7. Ability to run standard unmodified Socket library based utilities like sftp, scp, etc on RapidIO fabric  
8. Ability to run standard communication middlewares like Hadoop or OpenMPI
9. Sustained application level throughput measured by standard iperf utility in excess of 14 Gbps
10. Application point to point latency of < 1-2 usec for small packet sizes.
11. Support for 100s of nodes
12. Support of x86 architecture. Support on other architectures can be added on request basis.
