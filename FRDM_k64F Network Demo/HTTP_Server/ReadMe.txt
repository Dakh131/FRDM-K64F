-------------------------------------------------------------------------------
Author: David Khosravi (davidkhosravi@yahoo.se)
		Date:     2019-07-22
  
-------------------------------------------------------------------------------
 
 This Demo is build base on Keil Demo for HTTP sever IPV4/IPV6
 
Pack: Keil.MDK-Pro:Network:7.11.0
Pack: Keil.ARM_Compiler.1.6.1
Pack: NXP.MK64F12_DFP.12.0.0
	NXP:CMSIS Driver:Ethernet:enet_cmsis:2.0.0
	
ARM::CMSIS:RTOS:Keil RTX:4.82.0
Keil.ARM Compiler::Compiler:I/O:STDOUT:ITM:1.2.0 
Keil.MDK-Pro::Network:CORE:IPv4/IPv6 Debug:7.11.0
Keil.MDK-Pro::Network:Interface:ETH:7.11.0
Keil.MDK-Pro::Network:Service:Web Server Compact:HTTP:7.11.0
Keil.MDK-Pro::Network:Socket:TCP:7.11.0
Keil.MDK-Pro::Network:Socket:UDP:7.11.0

-------------------------------------------------------------------------------
known issues:
1) Event Recorder not located in uninitialized memory!
