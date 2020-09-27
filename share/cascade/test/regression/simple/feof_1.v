integer fd = $fopen("share/cascade/test/regression/simple/feof_1.dat");

integer COUNT = 0;
reg r = 0;

always @(posedge clock.val) begin
  // Pre-JIT-transition logic
  if ($target() == "sw") begin
    COUNT <= COUNT + 1;
    case (COUNT) 
      0: $write($feof(fd)); // No operations have been performed: 0
      1: $fread(fd, r);     // Sets eof state to 1
      2: $write($feof(fd)); // Stream is in eof state: 1
    endcase
  end 
  // Post-transition logic
  else begin
    if ($feof(fd)) $write(1); else $write(0); // feof in conditional statement post-jit: 1
    $write($feof(fd));                        // feof in system task post-jit: 1
    $finish; 
  end
end
