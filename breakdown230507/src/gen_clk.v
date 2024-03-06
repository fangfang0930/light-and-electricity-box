//名称：时钟生成模块
//模块名称：gen_clk.v
//日期：2018-11-20
//初始者：by cay
//功能：生成1Khz和50Hz时钟
//备注：

module gen_clk(
  input wire i_clk_50M,  //50Mhz时钟输入，周期0.02us
  output reg o_clk_1K,   //生成1khz时钟,周期1ms
  output reg o_clk_50    //生成50hz时钟，周期20ms
);

reg[15:0] c_1K = 16'd0;
reg[18:0] c_50 = 19'd0;
reg clk_1K = 1'b0;
reg clk_50 = 1'b0;
//生成1Khz时钟
always@(posedge i_clk_50M) begin
  if(c_1K >= 16'd24999) begin
    clk_1K <= ~clk_1K;
	 c_1K <= 1'd0;
  end
  else
    c_1K <= c_1K + 1'd1;
end
//生成50hz时钟
always@(posedge i_clk_50M) begin
  if(c_50 >= 19'd499999) begin
    clk_50 <= ~clk_50;
	 c_50 <= 1'd0;
  end
  else
    c_50 <= c_50 + 1'd1;
end

always@(posedge i_clk_50M) begin
  o_clk_1K <= clk_1K;
  o_clk_50 <= clk_50;  
end
endmodule
