`timescale 1ns/1ps
module tb;
reg clk=0,sys=0;always #7.8125 clk=~clk;always #5 sys=~sys;
reg rst=0,swe=1;reg [7:0] bus=0;
wire [31:0] collected,observed;wire push;
reg available=0;
always @(posedge sys) if(!rst)available<=0;else if(push)available<=1;
smi_ctrl smi(.i_rst_b(rst),.i_sys_clk(sys),.i_ioc(5'd0),.i_data_in(8'd0),.i_cs(1'b0),.i_fetch_cmd(1'b0),.i_load_cmd(1'b0),.i_rx_fifo_pulled_data(32'd0),.i_rx_fifo_empty(1'b1),.i_tx_fifo_full(1'b0),.o_tx_fifo_push(push),.o_tx_fifo_pushed_data(collected),.i_smi_soe_se(1'b1),.i_smi_swe_srw(swe),.i_smi_data_in(bus));
wire [1:0] serial;
lvds_tx tx(.i_rst_b(rst),.i_ddr_clk(clk),.i_fifo_empty(!available),.i_fifo_data(collected),.i_sample_gap(4'd3),.i_tx_state(1'b1),.i_debug_lb(1'b0),.o_ddr_data(serial));
lvds_rx rx(.i_rst_b(rst),.i_ddr_clk(clk),.i_ddr_data(serial),.i_fifo_full(1'b1),.i_sync_input(1'b0),.o_observed_word(observed));
reg cs=0,fetch=0;reg [4:0] addr=0;wire [7:0] result;
sys_ctrl ctrl(.i_rst_b(rst),.i_sys_clk(sys),.i_observed_rx({observed[29:17],observed[13:1]}),.i_observed_tx({collected[29:17],collected[13:1]}),.i_ioc(addr),.i_data_in(8'd0),.i_cs(cs),.i_fetch_cmd(fetch),.i_load_cmd(1'b0),.o_data_out(result));
task byte_out(input[7:0] value);
begin @(negedge sys);bus=value;repeat(5)@(negedge sys);swe=0;repeat(6)@(negedge sys);swe=1;repeat(8)@(negedge sys);end endtask
task get(input[4:0] a,output[7:0] value);
begin @(negedge sys);addr=a;cs=1;fetch=1;@(negedge sys);value=result;cs=0;fetch=0;end endtask
reg [7:0] value;reg [31:0] got_tx,got_rx,expected;integer bitno,k,n;reg [12:0] ii,qq;
task pattern(input[12:0] i,input[12:0] q);
begin
 expected={2'b10,i,1'b1,2'b01,q,1'b0};
 for(n=0;n<5;n=n+1)begin
  byte_out({3'b111,i[12:8]});byte_out({1'b0,i[7:1]});
  byte_out({1'b0,i[0],q[12:7]});byte_out({1'b0,q[6:0]});
 end
 repeat(800)@(negedge sys);
 get(8,value);if(value!==8'ha5)$fatal(1,"marker %h",value);
 got_tx=collected;
 for(k=0;k<4;k=k+1)begin get(9+k,value);got_rx[8*k+:8]=value;end
 if(got_tx!==expected)$fatal(1,"collector i=%h q=%h got=%h expected=%h",i,q,got_tx,expected);
 if(got_rx!=={6'b0,i,q})$fatal(1,"echo i=%h q=%h got=%h",i,q,got_rx);

end endtask
initial begin
 repeat(8)@(negedge sys);rst=1;
 for(bitno=0;bitno<13;bitno=bitno+1)begin pattern(13'b1<<bitno,0);pattern(0,13'b1<<bitno);end
 pattern(13'h1555,13'h0aaa);pattern(13'h1fff,13'h1fff);pattern(13'h1000,13'h1000);pattern(0,0);
 $display("PASS: 30 bit patterns through SMI collector, LVDS loopback and constant-pattern SPI readback; RX FIFO full");$finish;
end
initial begin #5000000;$fatal(1,"timeout");end
endmodule
