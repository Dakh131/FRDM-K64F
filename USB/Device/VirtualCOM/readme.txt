001
Baserat på hello world genererad med MCUXpresso Config Tools.
Allt från MXUXpresso utom board, clock_config och pin_mux utbytt mot motsvarande från RTE.
Minimal uppsättning komponenter från RTE används.
RTX fungerar.

-----------------------------------------------------------------------------
                          Known issues
-----------------------------------------------------------------------------

- Warning: Event Recorder not located in uninitialized memory! DK(20190726)
- ITM stdout (printf) fungerar inte tillsammans med nxp debug console. DK(20190726)

