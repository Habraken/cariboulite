module sequence_check #(parameter ALLOW_HOLD=0)(input clk,input rst_b,input valid,input [25:0] data,output [25:0] status);
wire [12:0] i=data[25:13],q=data[12:0];
wire [12:0] want_q={i[7:0],i[12:8]} ^ 13'h1555;
reg active,error;reg [12:0] previous,count;
wire done=&count;
assign status={10'b0,done,error,active,count};
always @(posedge clk or negedge rst_b) begin
 if(!rst_b)begin active<=0;error<=0;previous<=0;count<=0;end
 else if(valid && !done) begin
  if(!active)begin if(q==want_q)begin active<=1;previous<=i;end end
  else if(ALLOW_HOLD && i==previous && q==want_q)begin end
  else begin
   if(q!=want_q || i != ((previous+13'd73)&13'h1fff))error<=1;
   previous<=i;count<=count+1'b1;
  end
 end
end
endmodule
