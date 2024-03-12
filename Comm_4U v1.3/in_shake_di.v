//名称：DI输入防抖模块
//模块名称：in_shake_di.v
//日期：2018-11-20
//初始者：by cay
//功能：对输入进行防抖处理，避免误动，防抖计数
//备注：

module in_shake_di(
input	wire clk,  // 10Mhz时钟输入
input	wire in_i, // 待消抖信号输入
output reg in_o  // 消抖后输出
);

parameter io_shake = 8'd100;  // 定义常量10us
reg[7:0]	c_num;  // 高电平计数
reg[7:0]	o_num;  // 低电平计数

always @(posedge clk )
if(in_i) begin
	o_num <= 8'd0;
	c_num <= c_num + 8'd1;
	if(c_num >= io_shake) begin
		c_num <= io_shake;
		in_o <= 1'b1;
	end
end
else begin
	c_num <= 8'd0;
	o_num <=o_num + 8'd1;
	if(o_num >= io_shake) begin
		o_num <= io_shake;
		in_o <= 1'b0;
	end
end
endmodule