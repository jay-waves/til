Effective Data-Race Detection for the Kernel. 

```
AtPeriodicIntervals() {
    // determine k based on desired
    // memory access sampling rate
    repeat k times {
        pc = RandomlyChosenMemoryAccess();
        SetCodeBreakpoint( pc );
    }
}

OnCodeBreakpoint( pc ) {
    // disassemble the instruction at pc
    (loc, size, isWrite) = disasm( pc );

    DetectConflicts(loc, size, isWrite);

    // set another code break point
    pc = RandomlyChosenMemoryAccess();
    SetCodeBreakpoint( pc );
}

DetectConflicts( loc, size, isWrite ) {
    temp = read( loc, size );

    if ( isWrite )
        SetDataBreakpointRW( loc, size );
    else
        SetDataBreakpointW( loc, size );

    delay();

    ClearDataBreakpoint( loc, size );

    temp' = read( loc, size );

    if ( temp != temp' ||
         data_breakpoint_fired )
        ReportDataRace( );
}
```