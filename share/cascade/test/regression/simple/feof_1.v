integer fd = $fopen("share/cascade/test/regression/simple/feof_1.dat");

integer COUNT = 0;
reg r = 0;

always @(posedge clock.val) begin
  COUNT <= COUNT + 1;
  case (COUNT) 
    0:      $write($feof(fd)); // No operations have been performed: 0
    1:      $fread(fd, r);     // Sets eof state to 1
    2:      $write($feof(fd)); // Stream is in eof state: 1
    100000: if ($feof(fd)) $write(1); else $write(0); // feof in conditional statement post-jit: 1
    100001: $write($feof(fd)); // feof in system task post-jit: 1
    100002: $finish; 
  endcase
end
