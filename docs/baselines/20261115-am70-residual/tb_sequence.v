`timescale 1ns/1ps
`include "sequence_check.v"
module tb;
reg clk=0;always #5 clk=~clk;
reg rst=0,valid=0;reg [25:0] data=0;wire [25:0] tx,rx;
sequence_check t(clk,rst,valid,data,tx);
sequence_check #(.ALLOW_HOLD(1)) r(clk,rst,valid,data,rx);
reg [12:0] value;integer n;
task send;input [12:0] i;input bad;begin @(negedge clk);valid=1;data={i,({i[7:0],i[12:8]} ^ 13'h1555 ^ bad)};@(negedge clk);valid=0;end endtask
task reset;begin @(negedge clk);rst=0;valid=0;@(negedge clk);rst=1;end endtask
initial begin
 reset;value=13'h123;
 for(n=0;n<8192;n=n+1)begin send(value,0);value=value+73;end
 if(tx!==26'hbfff || rx!==26'hbfff)$fatal(1,"clean stream %h %h",tx,rx);
 reset;send(1,0);send(1,0);send(74,0);
 if(!tx[14] || rx[14] || rx[12:0]!=1)$fatal(1,"duplicate policy");
 reset;send(1,0);send(74,1);
 if(!tx[14] || !rx[14])$fatal(1,"Q corruption undetected");
 reset;send(1,0);send(147,0);
 if(!tx[14] || !rx[14])$fatal(1,"skipped sample undetected");
 $display("PASS clean full-code cycle, duplicate policy, Q corruption, skipped sample");$finish;
end
endmodule
