//
//
//
//
//

module bk_test_1chne(
  input wire i_clk_25m,  //25m时钟输入
  output reg o_bk_pulse   //反馈信号
);
reg [18:0]cnt = 18'd0;  //累加计数
parameter cnt_time1 = 8'd50;  //延时1 1s
parameter cnt_time2 = 8'd250;  //延时2 5s=250*0.02s
always@(posedge i_clk_25m) begin
  cnt <= cnt + 1'd1;
  if(cnt <= 18'd250000) o_bk_pulse <= 1'b0;  
  else if(cnt <= 18'd250875) o_bk_pulse <= 1'b1;   
//  else if(cnt <= 18'd250875) o_bk_pulse <= 1'b1;
  else if(cnt <= 18'd500000) o_bk_pulse <= 1'b0; 
  else begin
    cnt <= 18'd0;
    o_bk_pulse <= 1'b0;
  end
end
endmodule
