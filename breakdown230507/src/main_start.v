//
//
//
//
//

module main_start(
  input wire i_clk_50,  //50Hz时钟输入
  output reg o_start1,   //启动主程序标志1
  output reg o_start2   //启动主程序标志2
);
reg[7:0] cnt = 8'd0;  //累加计数
parameter cnt_time1 = 8'd50;  //延时1 1s
parameter cnt_time2 = 8'd250;  //延时2 5s=250*0.02s
always@(posedge i_clk_50) begin
  cnt <= cnt + 1'd1;
  if(cnt < cnt_time1) begin
    o_start1 <= 1'b0; 
	 o_start2 <= 1'b0;   
  end
  else if(cnt == cnt_time1)
	 o_start1 <= 1'b1;
  else if(cnt >= cnt_time2) begin
    cnt <= cnt_time2;
    o_start2 <= 1'b1;
  end
end
endmodule
