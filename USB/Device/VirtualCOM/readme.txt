001
Baserat p� hello world genererad med MCUXpresso Config Tools.
Allt fr�n MXUXpresso utom board, clock_config och pin_mux utbytt mot motsvarande fr�n RTE.
Minimal upps�ttning komponenter fr�n RTE anv�nds.
RTX fungerar.

-----------------------------------------------------------------------------
                          Known issues
-----------------------------------------------------------------------------

- Warning: Event Recorder not located in uninitialized memory! DK(20190726)
	-har �tg�rdast med en ny scatter file som DavidF har skrivit. DK(20190808)
- ITM stdout (printf) fungerar inte tillsammans med nxp debug console. DK(20190726)
	-Har �tg�rdat genom att definera 
		__MCUXPRESSO,
		SDK_DEBUGCONSOLE=DEBUGCONSOLE_REDIRECT_TO_TOOLCHAIN . DK(20190726)
		



